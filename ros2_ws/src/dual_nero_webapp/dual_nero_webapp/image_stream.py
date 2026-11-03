"""Latest-frame image buffers for ROS camera topics.

The webapp subscribes to the RealSense nodes that are already running in the
workspace. Keeping capture ownership in ``realsense2_camera`` means the
browser dashboard and calibration/data-collection tools can run together.
"""
from __future__ import annotations

import collections
import threading
import time
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image


def _unpack_image(msg: Image, channels: int, dtype: np.dtype) -> np.ndarray:
    """Copy a ROS Image payload into a ``height x width x channels`` array."""
    itemsize = np.dtype(dtype).itemsize
    if msg.step < msg.width * channels * itemsize:
        raise ValueError(
            f'image step {msg.step} is too small for {msg.width}x{msg.height} '
            f'{msg.encoding}'
        )
    raw = np.frombuffer(msg.data, dtype=dtype)
    row_items = msg.step // itemsize
    rows = raw.reshape((msg.height, row_items))
    rows = rows[:, : msg.width * channels]
    if channels == 1:
        return rows.reshape((msg.height, msg.width)).copy()
    return rows.reshape((msg.height, msg.width, channels)).copy()


def image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    """Convert common color encodings to an OpenCV BGR image."""
    encoding = msg.encoding.lower()
    try:
        if encoding in ('rgb8', 'bgr8'):
            image = _unpack_image(msg, 3, np.uint8)
            if encoding == 'rgb8':
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            return image
        if encoding in ('rgba8', 'bgra8'):
            image = _unpack_image(msg, 4, np.uint8)
            code = cv2.COLOR_RGBA2BGR if encoding == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(image, code)
        if encoding in ('yuv422_yuy2', 'yuyv', 'yuv422'):
            image = _unpack_image(msg, 2, np.uint8)
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)
        if encoding in ('mono8', '8uc1'):
            mono = _unpack_image(msg, 1, np.uint8)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
    except (ValueError, cv2.error):
        return None
    return None


def depth_to_bgr(
    msg: Image,
    depth_min_m: float = 0.05,
    depth_max_m: float = 2.0,
) -> Optional[np.ndarray]:
    """Colorize a 16UC1 millimetre or 32FC1 metre depth image."""
    encoding = msg.encoding.lower()
    try:
        if encoding in ('16uc1', 'mono16'):
            depth_m = _unpack_image(msg, 1, np.uint16).astype(np.float32) * 0.001
        elif encoding == '32fc1':
            depth_m = _unpack_image(msg, 1, np.float32)
        else:
            return None
    except (ValueError, cv2.error):
        return None

    valid = np.isfinite(depth_m) & (depth_m > 0.0)
    if not np.any(valid):
        return np.zeros((msg.height, msg.width, 3), dtype=np.uint8)

    span = max(depth_max_m - depth_min_m, 1e-6)
    normalized = np.clip((depth_m - depth_min_m) / span, 0.0, 1.0)
    normalized[~valid] = 0.0
    gray = (normalized * 255.0).astype(np.uint8)
    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


def _jpeg_shape(data: bytes) -> Optional[Tuple[int, int]]:
    """Read width and height from a JPEG SOF segment without decoding it."""
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


class ImageStream:
    """Thread-safe latest-frame holder with lazy JPEG encoding."""

    def __init__(
        self,
        name: str,
        label: str,
        mode: str,
        depth_min_m: float = 0.05,
        depth_max_m: float = 2.0,
    ) -> None:
        self.name = name
        self.label = label
        self.mode = mode
        self.depth_min_m = depth_min_m
        self.depth_max_m = depth_max_m

        self._lock = threading.Lock()
        self._encode_lock = threading.Lock()
        self._image: Optional[Image] = None
        self._shape: Optional[Tuple[int, int]] = None
        self._jpeg: Optional[bytes] = None
        self._jpeg_seq = -1
        self._seq = 0
        self._stamp = 0.0
        self._frame_times: Deque[float] = collections.deque(maxlen=60)
        self._bad_encoding_logged = False

    def update(self, msg: Image) -> None:
        encoding = msg.encoding.lower()
        supported = (
            encoding in ('16uc1', 'mono16', '32fc1')
            if self.mode == 'depth'
            else encoding in (
                'rgb8', 'bgr8', 'rgba8', 'bgra8', 'yuv422_yuy2', 'yuyv',
                'yuv422', 'mono8', '8uc1')
        )
        if not supported:
            if not self._bad_encoding_logged:
                self._bad_encoding_logged = True
                # Avoid depending on rclpy logger in this small helper.
                print(
                    f'[dual_nero_webapp] unsupported {self.mode} encoding '
                    f'{msg.encoding!r} for {self.name}',
                    flush=True,
                )
            return
        now = time.monotonic()
        with self._lock:
            self._image = msg
            self._shape = (msg.height, msg.width)
            self._seq += 1
            self._stamp = now
            self._frame_times.append(now)
            self._jpeg = None
            self._jpeg_seq = -1

    def latest_jpeg(self) -> Tuple[Optional[bytes], int]:
        with self._lock:
            if self._image is None:
                return None, self._seq
            if self._jpeg is not None and self._jpeg_seq == self._seq:
                return self._jpeg, self._seq
            seq = self._seq
            image = self._image

        with self._encode_lock:
            with self._lock:
                if self._jpeg is not None and self._jpeg_seq == self._seq:
                    return self._jpeg, self._seq
                seq = self._seq
                image = self._image
            if image is None:
                return None, seq
            frame = (
                depth_to_bgr(image, self.depth_min_m, self.depth_max_m)
                if self.mode == 'depth'
                else image_to_bgr(image)
            )
            if frame is None:
                return None, seq
            ok, encoded = cv2.imencode(
                '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78]
            )
            if not ok:
                return None, seq
            jpeg = encoded.tobytes()
            with self._lock:
                if self._seq == seq:
                    self._jpeg = jpeg
                    self._jpeg_seq = seq
            return jpeg, seq

    def sequence(self) -> int:
        with self._lock:
            return self._seq

    def info(self) -> dict:
        with self._lock:
            now = time.monotonic()
            recent = [t for t in self._frame_times if now - t <= 2.0]
            fps = 0.0
            if len(recent) >= 2:
                fps = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
            shape = list(self._shape) if self._shape is not None else None
            return {
                'name': self.name,
                'label': self.label,
                'mode': self.mode,
                'seq': self._seq,
                'fps': round(fps, 1),
                'shape': shape,
                'age_sec': None if self._stamp == 0.0 else round(now - self._stamp, 2),
            }


class CompressedImageStream:
    """Thread-safe latest-frame holder for already encoded JPEG messages."""

    def __init__(self, name: str, label: str, mode: str) -> None:
        self.name = name
        self.label = label
        self.mode = mode

        self._lock = threading.Lock()
        self._image: Optional[CompressedImage] = None
        self._shape: Optional[Tuple[int, int]] = None
        self._jpeg: Optional[bytes] = None
        self._jpeg_seq = -1
        self._seq = 0
        self._stamp = 0.0
        self._frame_times: Deque[float] = collections.deque(maxlen=60)
        self._bad_format_logged = False

    def update(self, msg: CompressedImage) -> None:
        image_format = msg.format.lower()
        if 'jpeg' not in image_format and 'jpg' not in image_format:
            if not self._bad_format_logged:
                self._bad_format_logged = True
                print(
                    f'[dual_nero_webapp] unsupported compressed format '
                    f'{msg.format!r} for {self.name}',
                    flush=True,
                )
            return
        data = bytes(msg.data)
        now = time.monotonic()
        with self._lock:
            self._image = msg
            self._shape = _jpeg_shape(data)
            self._seq += 1
            self._stamp = now
            self._frame_times.append(now)
            self._jpeg = None
            self._jpeg_seq = -1

    def latest_jpeg(self) -> Tuple[Optional[bytes], int]:
        with self._lock:
            if self._image is None:
                return None, self._seq
            if self._jpeg is not None and self._jpeg_seq == self._seq:
                return self._jpeg, self._seq
            seq = self._seq
            image = self._image

        jpeg = bytes(image.data)
        with self._lock:
            if self._seq == seq:
                self._jpeg = jpeg
                self._jpeg_seq = seq
        return jpeg, seq

    def sequence(self) -> int:
        with self._lock:
            return self._seq

    def info(self) -> dict:
        with self._lock:
            now = time.monotonic()
            recent = [t for t in self._frame_times if now - t <= 2.0]
            fps = 0.0
            if len(recent) >= 2:
                fps = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
            shape = list(self._shape) if self._shape is not None else None
            return {
                'name': self.name,
                'label': self.label,
                'mode': self.mode,
                'seq': self._seq,
                'fps': round(fps, 1),
                'shape': shape,
                'age_sec': None if self._stamp == 0.0 else round(now - self._stamp, 2),
            }
