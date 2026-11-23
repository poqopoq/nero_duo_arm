from __future__ import annotations

import json
import os
import socket
import struct
import tempfile
import threading
import time
from pathlib import Path

import pytest

from dual_nero_webapp.inference_controller import InferenceController, InferenceError


FAKE_ROLLOUT = """#!/bin/sh
if [ ! -t 0 ]; then
  echo "not a tty"
  exit 9
fi
echo "Interactive rollout session — fake"
while IFS= read -r line; do
  echo "received: $line"
  case "$line" in
    /start*) echo "Rollout running — task 'fake'." ;;
    /reset*)
      echo "Resetting — returning the robot to its reset position..."
      echo "Robot reset — holding at reset position. /start to run."
      ;;
    /subtask*) echo "Task changed" ;;
    /stop*)
      echo "Interactive session ended"
      exit 0
      ;;
  esac
done
"""


def wait_until(predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError('condition was not reached before timeout')


@pytest.fixture()
def rollout_script(tmp_path: Path) -> Path:
    path = tmp_path / 'fake_joint16_rollout.sh'
    path.write_text(FAKE_ROLLOUT)
    path.chmod(0o755)
    return path


def make_controller(script: Path, bridge_socket: str) -> InferenceController:
    return InferenceController(
        str(script),
        bridge_socket=bridge_socket,
        default_task='Use the right arm to put apple into white box.',
        preflight=False,
        stop_grace_sec=2.0,
        bridge_probe_interval_sec=0.1,
    )


def test_interactive_lifecycle_uses_a_pty_and_never_auto_starts(rollout_script: Path, tmp_path: Path):
    controller = make_controller(rollout_script, str(tmp_path / 'missing.sock'))
    try:
        response = controller.launch('Use the right arm to put apple into white box.')
        assert response['ok']
        wait_until(lambda: controller.status()['state'] == 'idle')
        status = controller.status(include_log=True)
        assert status['manual_control_locked'] is True
        assert status['task'] == 'Use the right arm to put apple into white box.'
        assert not any('/start' in line for line in status['log'])

        assert controller.start()['ok']
        wait_until(lambda: controller.status()['state'] == 'running')
        assert controller.set_task('Use the right arm to put bottle into white box.')['ok']
        assert controller.reset()['ok']
        wait_until(lambda: controller.status()['state'] == 'idle')
        assert controller.stop(wait=True)['ok']
        wait_until(lambda: controller.status()['state'] == 'stopped')
        assert not controller.active
    finally:
        controller.shutdown()


def test_second_launch_is_rejected_while_process_is_alive(rollout_script: Path, tmp_path: Path):
    controller = make_controller(rollout_script, str(tmp_path / 'missing.sock'))
    try:
        controller.launch('task one')
        wait_until(lambda: controller.status()['state'] == 'idle')
        with pytest.raises(InferenceError, match='already running'):
            controller.launch('task two')
    finally:
        controller.stop(wait=True)
        controller.shutdown()


@pytest.mark.parametrize('task', ['', 'line one\nline two', 'x' * 1001])
def test_task_validation_rejects_empty_multiline_and_oversized_values(
    rollout_script: Path, tmp_path: Path, task: str
) -> None:
    controller = make_controller(rollout_script, str(tmp_path / 'missing.sock'))
    with pytest.raises(InferenceError):
        controller.launch(task)
    assert controller.status()['state'] == 'stopped'


class FakeBridge:
    def __init__(self, socket_path: Path) -> None:
        self.socket_path = socket_path
        self.fault = 'stop command received'
        self.gripper_motion_required = False
        self.gripper_ready = None
        self.gripper_health = None
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.server.bind(str(socket_path))
        self.server.listen(4)
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self) -> None:
        while True:
            try:
                conn, _ = self.server.accept()
            except OSError:
                return
            with conn:
                try:
                    raw = self._recv_exact(conn, 8)
                    header_size, payload_size = struct.unpack('!II', raw)
                    request = json.loads(self._recv_exact(conn, header_size).decode())
                    if payload_size:
                        self._recv_exact(conn, payload_size)
                    if request.get('cmd') == 'clear_fault':
                        previous = self.fault
                        self.fault = None
                        response = {
                            'ok': True,
                            'cleared': previous is not None,
                            'previous_fault': previous,
                        }
                    else:
                        response = self._ping()
                    body = json.dumps(response).encode()
                    conn.sendall(struct.pack('!II', len(body), 0) + body)
                except (OSError, ValueError, json.JSONDecodeError):
                    continue

    @staticmethod
    def _recv_exact(conn: socket.socket, size: int) -> bytes:
        data = b''
        while len(data) < size:
            chunk = conn.recv(size - len(data))
            if not chunk:
                raise ConnectionError
            data += chunk
        return data

    def _ping(self) -> dict:
        return {
            'ok': True,
            'schema': 'joint16',
            'execution_mode': 'stream',
            'dry_run': False,
            'real_motion_enabled': True,
            'motion_fault': self.fault,
            'gripper_motion_required': self.gripper_motion_required,
            'gripper_ready': self.gripper_ready,
            'gripper_health': self.gripper_health,
        }

    def close(self) -> None:
        self.server.close()
        try:
            os.unlink(self.socket_path)
        except FileNotFoundError:
            pass


def test_bridge_fault_is_visible_and_clearable_only_without_rollout(
    rollout_script: Path, tmp_path: Path
):
    socket_path = tmp_path / 'bridge.sock'
    bridge = FakeBridge(socket_path)
    controller = make_controller(rollout_script, str(socket_path))
    try:
        status = controller.status()
        assert status['bridge_fault'] == 'stop command received'
        assert status['can_clear_fault'] is True
        response = controller.clear_bridge_fault()
        assert response['ok']
        assert response['previous_fault'] == 'stop command received'
        assert controller.status()['bridge_fault'] is None

        controller.launch('task')
        wait_until(lambda: controller.status()['state'] == 'idle')
        with pytest.raises(InferenceError, match='stop the inference process'):
            controller.clear_bridge_fault()
    finally:
        controller.stop(wait=True)
        controller.shutdown()
        bridge.close()


def test_preflight_rejects_latched_gripper_before_launch(
    rollout_script: Path, tmp_path: Path, monkeypatch
):
    socket_path = tmp_path / 'bridge.sock'
    bridge = FakeBridge(socket_path)
    bridge.fault = None
    bridge.gripper_motion_required = True
    bridge.gripper_ready = False
    bridge.gripper_health = {
        'ready': False,
        'reason': 'right: emergency stop is latched',
        'sides': {},
    }
    controller = InferenceController(
        str(rollout_script),
        bridge_socket=str(socket_path),
        preflight=True,
    )
    monkeypatch.setattr(controller, '_port_listening', lambda: True)
    try:
        with pytest.raises(InferenceError, match='gripper driver is not ready'):
            controller.launch('task')
        assert not controller.active
    finally:
        controller.shutdown()
        bridge.close()
