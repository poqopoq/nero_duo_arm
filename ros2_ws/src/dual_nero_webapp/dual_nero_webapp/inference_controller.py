#!/usr/bin/env python3
"""Interactive joint16 rollout process control for the browser dashboard.

The rollout CLI is intentionally kept in its own process.  A pseudo-terminal is
used because the deployment wrapper refuses to start without a TTY, and because
the interactive session reads control commands from stdin.

This module has no ROS imports so its state machine can be tested independently
from the webapp node.
"""
from __future__ import annotations

import json
import os
import pty
import re
import signal
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Optional


_FRAME_HEADER = struct.Struct('!II')
_MAX_HEADER_BYTES = 1 << 20
_ANSI_ESCAPE = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')
_ACTIVE_STATES = {'loading', 'idle', 'starting', 'running', 'resetting', 'stopping'}


class InferenceError(RuntimeError):
    """A rejected or failed inference operation."""


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError('bridge socket closed while reading a frame')
        chunks.extend(chunk)
    return bytes(chunks)


def _send_bridge_request(sock: socket.socket, request: Dict[str, Any]) -> Dict[str, Any]:
    body = json.dumps(request, separators=(',', ':')).encode('utf-8')
    if len(body) > _MAX_HEADER_BYTES:
        raise ValueError('bridge request is too large')
    sock.sendall(_FRAME_HEADER.pack(len(body), 0))
    sock.sendall(body)
    raw_header = _recv_exact(sock, _FRAME_HEADER.size)
    header_size, payload_size = _FRAME_HEADER.unpack(raw_header)
    if header_size > _MAX_HEADER_BYTES or payload_size:
        raise ValueError('unexpected bridge response frame')
    response = json.loads(_recv_exact(sock, header_size).decode('utf-8'))
    if not isinstance(response, dict):
        raise ValueError('bridge response must be a JSON object')
    return response


class InferenceController:
    """Own one interactive rollout subprocess and expose start/stop/reset/task."""

    def __init__(
        self,
        rollout_script: str,
        bridge_socket: str = '/tmp/nero_bimanual_vla_bridge.sock',
        default_task: str = 'Use the right arm to put apple into white box.',
        *,
        policy_port: int = 7447,
        log_limit: int = 240,
        stop_grace_sec: float = 10.0,
        bridge_probe_interval_sec: float = 1.0,
        preflight: bool = True,
    ) -> None:
        self.rollout_script = str(Path(rollout_script).expanduser())
        self.bridge_socket = str(Path(bridge_socket).expanduser())
        self.default_task = self._validate_task(default_task)
        self.policy_port = int(policy_port)
        self.stop_grace_sec = max(1.0, float(stop_grace_sec))
        self.bridge_probe_interval_sec = max(0.1, float(bridge_probe_interval_sec))
        self.preflight_enabled = bool(preflight)

        self._lock = threading.RLock()
        self._log: Deque[str] = deque(maxlen=max(40, int(log_limit)))
        self._process: Optional[subprocess.Popen[bytes]] = None
        self._master_fd: Optional[int] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._terminate_thread: Optional[threading.Thread] = None
        self._state = 'stopped'
        self._error: Optional[str] = None
        self._exit_code: Optional[int] = None
        self._launched_at: Optional[float] = None
        self._ready_at: Optional[float] = None
        self._stop_requested = False
        self._task = self.default_task
        self._initial_task = self.default_task
        self._bridge_cache: Dict[str, Any] = {
            'reachable': False,
            'error': 'bridge has not been probed yet',
            'motion_fault': None,
            'schema': None,
            'execution_mode': None,
            'dry_run': None,
            'gripper_motion_required': False,
            'gripper_ready': None,
            'gripper_health': None,
        }
        self._bridge_cache_at = 0.0

    @staticmethod
    def _validate_task(task: Any) -> str:
        value = str(task or '').strip()
        if not value:
            raise ValueError('task must not be empty')
        if any(character in value for character in ('\x00', '\r', '\n')):
            raise ValueError('task must be a single line')
        if len(value) > 1000:
            raise ValueError('task is too long (maximum 1000 characters)')
        return value

    @property
    def active(self) -> bool:
        """True while a rollout subprocess is alive and manual motion must be blocked."""
        with self._lock:
            return self._process_alive_locked()

    def _process_alive_locked(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _append_log_locked(self, line: str) -> None:
        text = _ANSI_ESCAPE.sub('', line).replace('\r', '').strip()
        if not text:
            return
        stamped = time.strftime('%H:%M:%S') + '  ' + text
        self._log.append(stamped)

        if 'Interactive rollout session' in text:
            self._state = 'idle'
            self._ready_at = time.monotonic()
            self._error = None
        elif 'Rollout running —' in text:
            self._state = 'running'
            self._error = None
        elif 'Rollout run ended.' in text:
            self._state = 'idle'
        elif 'Resetting — returning the robot' in text:
            self._state = 'resetting'
            self._error = None
        elif (
            'Robot reset — holding at reset position' in text
            or 'Robot reset — holding at initial position' in text
        ):
            self._state = 'idle'
            self._error = None
        elif (
            'Robot paused — no reset position available' in text
            or 'Robot paused — no initial position captured' in text
        ):
            self._state = 'idle'
            self._error = 'reset skipped: no reset position is available'
        elif 'Reset FAILED' in text:
            self._state = 'idle'
            self._error = 'reset failed; verify the robot pose before /start'
        elif 'Inference engine failed' in text or 'Rollout strategy failed' in text:
            self._state = 'fault'
            self._error = text
        elif 'Interactive session ended' in text:
            self._state = 'stopping'

    def _reader_loop(self, master_fd: int, process: subprocess.Popen[bytes]) -> None:
        pending = b''
        try:
            while True:
                try:
                    chunk = os.read(master_fd, 4096)
                except OSError:
                    break
                if not chunk:
                    break
                pending += chunk
                while b'\n' in pending:
                    raw_line, pending = pending.split(b'\n', 1)
                    with self._lock:
                        self._append_log_locked(
                            raw_line.decode('utf-8', errors='replace'))
            if pending:
                with self._lock:
                    self._append_log_locked(
                        pending.decode('utf-8', errors='replace'))
        finally:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                pass
            try:
                os.close(master_fd)
            except OSError:
                pass
            with self._lock:
                self._master_fd = None
                self._exit_code = process.poll()
                was_ready = self._ready_at is not None
                if self._state != 'fault':
                    if self._stop_requested:
                        self._state = 'stopped'
                    elif self._exit_code not in (0, None) or not was_ready:
                        self._state = 'fault'
                        if self._error is None:
                            self._error = (
                                'rollout exited before the interactive session was ready'
                                if not was_ready else
                                f'rollout exited with code {self._exit_code}')
                    else:
                        self._state = 'stopped'
                if self._process is process:
                    self._process = None
                self._bridge_cache_at = 0.0

    def _bridge_request(self, request: Dict[str, Any], timeout: float = 0.6) -> Dict[str, Any]:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            sock.connect(self.bridge_socket)
            return _send_bridge_request(sock, request)

    def _bridge_status(self, force: bool = False) -> Dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            cache_age = now - self._bridge_cache_at
            if not force and cache_age < self.bridge_probe_interval_sec:
                return dict(self._bridge_cache)
        try:
            response = self._bridge_request({'cmd': 'ping'})
            result = {
                'reachable': True,
                'error': None if response.get('ok') else response.get('error'),
                'motion_fault': response.get('motion_fault'),
                'schema': response.get('schema'),
                'execution_mode': response.get('execution_mode'),
                'dry_run': response.get('dry_run'),
                'real_motion_enabled': response.get('real_motion_enabled'),
                'gripper_motion_required': response.get(
                    'gripper_motion_required', False),
                'gripper_ready': response.get('gripper_ready'),
                'gripper_health': response.get('gripper_health'),
            }
        except Exception as exc:  # noqa: BLE001
            result = {
                'reachable': False,
                'error': f'{type(exc).__name__}: {exc}',
                'motion_fault': None,
                'schema': None,
                'execution_mode': None,
                'dry_run': None,
                'real_motion_enabled': None,
                'gripper_motion_required': False,
                'gripper_ready': None,
                'gripper_health': None,
            }
        with self._lock:
            self._bridge_cache = result
            self._bridge_cache_at = time.monotonic()
        return dict(result)

    def _port_listening(self) -> bool:
        try:
            completed = subprocess.run(
                ['ss', '-ltn'],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.5,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return re.search(rf':{self.policy_port}\b', completed.stdout) is not None

    def preflight(self) -> Optional[str]:
        script = Path(self.rollout_script)
        if not script.is_file():
            return f'rollout script not found: {script}'
        if not os.access(script, os.X_OK):
            return f'rollout script is not executable: {script}'
        if not Path(self.bridge_socket).is_socket():
            return f'bridge socket is not available: {self.bridge_socket}'
        if not self._port_listening():
            return (
                f'policy endpoint is not listening on 127.0.0.1:{self.policy_port}; '
                'start the GPU server and SSH tunnel first')
        bridge = self._bridge_status(force=True)
        if not bridge['reachable']:
            return f"bridge is not reachable: {bridge['error']}"
        if bridge.get('schema') != 'joint16':
            return (
                f"bridge schema is {bridge.get('schema')!r}, expected 'joint16'; "
                'restart the bridge with action_schema:=joint16')
        if bridge.get('motion_fault'):
            return (
                f"bridge motion fault is latched: {bridge['motion_fault']}; "
                'clear the bridge fault before launching inference')
        gripper_error = self._gripper_preflight_error(bridge)
        if gripper_error is not None:
            return gripper_error
        return None

    @staticmethod
    def _gripper_preflight_error(bridge: Dict[str, Any]) -> Optional[str]:
        if not bridge.get('gripper_motion_required'):
            return None
        if bridge.get('gripper_ready') is True:
            return None
        health = bridge.get('gripper_health')
        if not isinstance(health, dict):
            return (
                'bridge did not report gripper driver health; restart the '
                'bridge with the current code before launching inference')
        reason = health.get('reason') or 'gripper health is unavailable'
        return (
            f'gripper driver is not ready: {reason}; clear the gripper fault, '
            'enable both motors, and verify width/status feedback before launching')

    def launch(self, task: Optional[str] = None) -> Dict[str, Any]:
        with self._lock:
            if self._process_alive_locked():
                raise InferenceError('an inference process is already running')
        try:
            selected_task = self._validate_task(task if task is not None else self.default_task)
        except ValueError as exc:
            raise InferenceError(str(exc)) from exc

        if self.preflight_enabled:
            error = self.preflight()
            if error is not None:
                raise InferenceError(error)

        master_fd, slave_fd = pty.openpty()
        environment = os.environ.copy()
        environment['JOINT16_TASK'] = selected_task
        environment.setdefault('PYTHONUNBUFFERED', '1')
        try:
            process = subprocess.Popen(
                [self.rollout_script],
                stdin=slave_fd,
                stdout=slave_fd,
                stderr=slave_fd,
                cwd=str(Path(self.rollout_script).resolve().parents[2]),
                env=environment,
                start_new_session=True,
                close_fds=True,
            )
        except Exception:
            os.close(master_fd)
            os.close(slave_fd)
            raise
        os.close(slave_fd)

        with self._lock:
            self._process = process
            self._master_fd = master_fd
            self._state = 'loading'
            self._error = None
            self._exit_code = None
            self._stop_requested = False
            self._launched_at = time.monotonic()
            self._ready_at = None
            self._task = selected_task
            self._initial_task = selected_task
            self._log.clear()
            self._append_log_locked(f'Launching: {self.rollout_script}')
            self._append_log_locked(f'Task: {selected_task}')
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                args=(master_fd, process),
                name='dual-nero-inference-reader',
                daemon=True,
            )
            self._reader_thread.start()

        # Surface immediate exec/preflight failures without holding the HTTP request
        # through model loading, which can legitimately take much longer.
        deadline = time.monotonic() + 0.35
        while time.monotonic() < deadline:
            with self._lock:
                if self._state in ('idle', 'fault', 'stopped'):
                    break
            time.sleep(0.02)
        with self._lock:
            if self._state == 'fault':
                message = self._error or 'rollout failed during launch'
                raise InferenceError(message)
            return self._result_locked(True, 'rollout process launched; /start has NOT been sent')

    def _write_command_locked(self, command: str) -> None:
        if not self._process_alive_locked() or self._master_fd is None:
            raise InferenceError('rollout process is not running')
        data = (command.rstrip('\n') + '\n').encode('utf-8')
        view = memoryview(data)
        while view:
            written = os.write(self._master_fd, view)
            view = view[written:]

    def _require_ready_locked(self) -> None:
        if not self._process_alive_locked():
            raise InferenceError('rollout process is not running; launch it first')
        if self._state not in ('idle', 'running'):
            raise InferenceError(f'rollout is {self._state}; wait for the interactive session to be ready')

    def _check_bridge_motion_fault(self) -> None:
        bridge = self._bridge_status(force=True)
        if bridge.get('motion_fault'):
            raise InferenceError(
                f"bridge motion fault is latched: {bridge['motion_fault']}; "
                'stop inference and clear the bridge fault')
        gripper_error = self._gripper_preflight_error(bridge)
        if gripper_error is not None:
            raise InferenceError(gripper_error)

    def start(self) -> Dict[str, Any]:
        with self._lock:
            self._require_ready_locked()
        self._check_bridge_motion_fault()
        with self._lock:
            self._require_ready_locked()
            self._write_command_locked('/start')
            self._state = 'starting'
            self._error = None
            self._append_log_locked('WebApp sent /start')
            return self._result_locked(True, 'sent /start; wait for the running status')

    def reset(self) -> Dict[str, Any]:
        with self._lock:
            self._require_ready_locked()
            self._write_command_locked('/reset')
            self._state = 'resetting'
            self._error = None
            self._append_log_locked('WebApp sent /reset')
            return self._result_locked(True, 'sent /reset; the robot is returning to the captured training pose')

    def set_task(self, task: str) -> Dict[str, Any]:
        try:
            selected_task = self._validate_task(task)
        except ValueError as exc:
            raise InferenceError(str(exc)) from exc
        with self._lock:
            self._require_ready_locked()
            self._write_command_locked('/subtask ' + selected_task)
            self._task = selected_task
            self._append_log_locked(f'WebApp sent /subtask {selected_task}')
            return self._result_locked(True, 'subtask sent; it applies from the next policy inference')

    def stop(self, *, wait: bool = False) -> Dict[str, Any]:
        with self._lock:
            if not self._process_alive_locked():
                if self._state not in ('fault', 'stopped'):
                    self._state = 'stopped'
                return self._result_locked(True, 'inference is already stopped')
            self._stop_requested = True
            self._state = 'stopping'
            try:
                self._write_command_locked('/stop')
            except InferenceError:
                pass
            self._append_log_locked('WebApp sent /stop')
            process = self._process
            if self._terminate_thread is None or not self._terminate_thread.is_alive():
                self._terminate_thread = threading.Thread(
                    target=self._force_stop_after_grace,
                    args=(process,),
                    name='dual-nero-inference-stop',
                    daemon=True,
                )
                self._terminate_thread.start()
            result = self._result_locked(
                True,
                'sent /stop; inference is shutting down. This closes the bridge motion gate.')
        if wait:
            deadline = time.monotonic() + self.stop_grace_sec + 1.0
            while time.monotonic() < deadline and self.active:
                time.sleep(0.05)
        return result

    def _force_stop_after_grace(self, process: subprocess.Popen[bytes]) -> None:
        deadline = time.monotonic() + self.stop_grace_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.05)
        self._signal_process_group(process, signal.SIGTERM)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.05)
        self._signal_process_group(process, signal.SIGKILL)

    @staticmethod
    def _signal_process_group(process: subprocess.Popen[bytes], sig: signal.Signals) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, sig)
        except (ProcessLookupError, PermissionError):
            try:
                process.send_signal(sig)
            except ProcessLookupError:
                pass

    def clear_bridge_fault(self) -> Dict[str, Any]:
        if self.active:
            raise InferenceError('stop the inference process before clearing the bridge fault')
        try:
            response = self._bridge_request({'cmd': 'clear_fault'}, timeout=6.0)
        except Exception as exc:  # noqa: BLE001
            raise InferenceError(f'bridge fault-clear request failed: {exc}') from exc
        if not response.get('ok'):
            raise InferenceError(
                response.get('error') or 'bridge rejected the fault-clear request')
        self._bridge_status(force=True)
        with self._lock:
            message = (
                'bridge fault cleared' if response.get('cleared') else
                'bridge had no latched motion fault')
            return self._result_locked(True, message, extra={
                'previous_fault': response.get('previous_fault'),
            })

    def shutdown(self) -> None:
        with self._lock:
            process = self._process
        if process is None or process.poll() is not None:
            return
        try:
            self.stop(wait=True)
        finally:
            with self._lock:
                process = self._process
            if process is not None and process.poll() is None:
                self._signal_process_group(process, signal.SIGTERM)

    def _result_locked(
        self,
        ok: bool,
        message: str,
        *,
        extra: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        result = {
            'ok': ok,
            'message': message,
            'state': self._state,
            'task': self._task,
        }
        if extra:
            result.update(extra)
        return result

    def status(
        self,
        *,
        include_log: bool = False,
        refresh_bridge: bool = True,
    ) -> Dict[str, Any]:
        with self._lock:
            active = self._process_alive_locked()
            state = self._state
            result: Dict[str, Any] = {
                'state': state,
                'active': active,
                'manual_control_locked': active,
                'busy': state in _ACTIVE_STATES,
                'can_launch': not active,
                'can_start': active and state == 'idle',
                'can_reset': active and state in ('idle', 'running'),
                'can_stop': active,
                'task': self._task,
                'initial_task': self._initial_task,
                'error': self._error,
                'exit_code': self._exit_code,
                'rollout_script': self.rollout_script,
                'policy_port': self.policy_port,
                'launched_at': self._launched_at,
                'ready_at': self._ready_at,
                'log_count': len(self._log),
            }
            if include_log:
                result['log'] = list(self._log)
        if refresh_bridge:
            bridge = self._bridge_status()
        else:
            with self._lock:
                bridge = dict(self._bridge_cache)
        result['bridge'] = bridge
        result['can_clear_fault'] = bool(
            not active and bridge.get('reachable') and bridge.get('motion_fault'))
        result['bridge_fault'] = bridge.get('motion_fault')
        return result
