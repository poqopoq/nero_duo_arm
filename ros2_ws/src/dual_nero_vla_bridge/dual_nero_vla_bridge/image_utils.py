"""Small helpers for forwarding camera frames without decoding them again."""
from __future__ import annotations

from typing import Optional


def jpeg_size(data: bytes) -> Optional[tuple[int, int]]:
    """Return ``(height, width)`` from a JPEG SOF segment."""
    if len(data) < 4 or data[:2] != b'\xff\xd8':
        return None
    offset = 2
    frame_markers = {
        0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
        0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
    }
    while offset + 1 < len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        marker = data[offset + 1]
        offset += 2
        if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
            continue
        if offset + 2 > len(data):
            break
        segment_length = (data[offset] << 8) | data[offset + 1]
        if segment_length < 2 or offset + segment_length > len(data):
            break
        if marker in frame_markers and segment_length >= 7:
            height = (data[offset + 3] << 8) | data[offset + 4]
            width = (data[offset + 5] << 8) | data[offset + 6]
            return height, width
        offset += segment_length
    return None
