"""Length-prefixed JSON/binary framing used by the local VLA bridge."""
from __future__ import annotations

import json
import socket
import struct
from typing import Any

MAX_HEADER_BYTES = 1 << 20
MAX_PAYLOAD_BYTES = 32 << 20
_FRAME_HEADER = struct.Struct('!II')


class ProtocolError(RuntimeError):
    """Raised when a peer sends a malformed or oversized frame."""


def _recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError('socket closed while reading a frame')
        chunks.extend(chunk)
    return bytes(chunks)


def send_frame(sock: socket.socket, header: dict[str, Any], payload: bytes = b'') -> None:
    header_bytes = json.dumps(header, separators=(',', ':')).encode('utf-8')
    if len(header_bytes) > MAX_HEADER_BYTES:
        raise ProtocolError(f'header is too large: {len(header_bytes)} bytes')
    if len(payload) > MAX_PAYLOAD_BYTES:
        raise ProtocolError(f'payload is too large: {len(payload)} bytes')
    sock.sendall(_FRAME_HEADER.pack(len(header_bytes), len(payload)))
    sock.sendall(header_bytes)
    if payload:
        sock.sendall(payload)


def recv_frame(sock: socket.socket) -> tuple[dict[str, Any], bytes]:
    raw_header = _recv_exact(sock, _FRAME_HEADER.size)
    header_size, payload_size = _FRAME_HEADER.unpack(raw_header)
    if header_size > MAX_HEADER_BYTES:
        raise ProtocolError(f'header is too large: {header_size} bytes')
    if payload_size > MAX_PAYLOAD_BYTES:
        raise ProtocolError(f'payload is too large: {payload_size} bytes')
    header = json.loads(_recv_exact(sock, header_size).decode('utf-8'))
    payload = _recv_exact(sock, payload_size) if payload_size else b''
    if not isinstance(header, dict):
        raise ProtocolError('frame header must be a JSON object')
    return header, payload

