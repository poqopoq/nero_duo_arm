"""Unix-socket client used by the LeRobot Python 3.12 process."""
from __future__ import annotations

import socket
import time
from typing import Any

from .protocol import ProtocolError, recv_frame, send_frame


class BridgeError(RuntimeError):
    """Raised when the ROS bridge is unavailable or rejects a request."""


class BridgeClient:
    def __init__(self, socket_path: str, timeout_s: float = 2.0) -> None:
        self.socket_path = socket_path
        self.timeout_s = timeout_s
        self._sock: socket.socket | None = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        if self._sock is not None:
            return
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout_s)
        try:
            sock.connect(self.socket_path)
        except OSError as exc:
            sock.close()
            raise BridgeError(f'cannot connect to bridge at {self.socket_path}: {exc}') from exc
        self._sock = sock

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None

    def _request_once(self, header: dict[str, Any], payload: bytes = b'') -> tuple[dict[str, Any], bytes]:
        self.connect()
        assert self._sock is not None
        send_frame(self._sock, header, payload)
        return recv_frame(self._sock)

    def request(self, command: str, **fields: Any) -> tuple[dict[str, Any], bytes]:
        request = {'cmd': command, **fields}
        last_error: Exception | None = None
        for attempt in range(2):
            try:
                header, payload = self._request_once(request)
                if not header.get('ok'):
                    raise BridgeError(str(header.get('error', 'bridge rejected request')))
                return header, payload
            except (BridgeError, OSError, ConnectionError, ProtocolError, TimeoutError) as exc:
                last_error = exc
                self.close()
                if attempt == 0:
                    time.sleep(0.05)
        raise BridgeError(f'bridge request {command!r} failed: {last_error}') from last_error

    def ping(self) -> dict[str, Any]:
        header, _ = self.request('ping')
        return header

    def get_observation(self) -> tuple[dict[str, Any], bytes]:
        return self.request('get_observation')

    def send_action(self, action: list[float]) -> dict[str, Any]:
        header, _ = self.request('send_action', action=action)
        return header

    def stop(self) -> dict[str, Any]:
        header, _ = self.request('stop')
        return header

