"""LeRobot Robot implementation backed by the NERO ROS 2 bridge."""
from __future__ import annotations

import logging

import cv2
import numpy as np
from lerobot.robots import Robot

from .bridge_client import BridgeClient, BridgeError
from .config_nero import NeroConfig

logger = logging.getLogger(__name__)

STATE_NAMES = (
    'ee.x', 'ee.y', 'ee.z',
    'ee.rx', 'ee.ry', 'ee.rz',
    'ee.gripper',
)


class Nero(Robot):
    """A 7-D absolute end-effector robot using the NERO ROS 2 stack.

    The state and action contract is an absolute TCP pose in metres followed by
    an axis-angle rotation in radians and a normalized gripper value where
    ``0`` is closed and ``1`` is open.
    """

    config_class = NeroConfig
    name = 'nero_ros'

    def __init__(self, config: NeroConfig) -> None:
        super().__init__(config)
        self.config = config
        self._client = BridgeClient(config.bridge_socket, config.request_timeout_s)
        self._connected = False
        self._camera_key = config.camera_key

    @property
    def observation_features(self) -> dict:
        features = {name: float for name in STATE_NAMES}
        features[self._camera_key] = (
            self.config.image_height,
            self.config.image_width,
            3,
        )
        return features

    @property
    def action_features(self) -> dict:
        return {name: float for name in STATE_NAMES}

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def configure(self) -> None:
        return None

    def calibrate(self) -> None:
        return None

    def connect(self, calibrate: bool = True) -> None:
        if self._connected:
            return
        try:
            status = self._client.ping()
        except BridgeError as exc:
            raise ConnectionError(str(exc)) from exc
        if status.get('side') != self.config.side:
            self._client.close()
            raise ConnectionError(
                f'bridge side is {status.get("side")!r}, expected {self.config.side!r}'
            )
        if calibrate:
            self.calibrate()
        self._connected = True
        logger.info('Connected to NERO bridge at %s', self.config.bridge_socket)

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError('Nero robot is not connected')

    def _decode_image(self, payload: bytes) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('bridge returned an image that could not be decoded')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        expected = (self.config.image_height, self.config.image_width)
        if image.shape[:2] != expected:
            image = cv2.resize(image, (expected[1], expected[0]), interpolation=cv2.INTER_AREA)
        if image.dtype != np.uint8:
            image = image.astype(np.uint8)
        return np.ascontiguousarray(image)

    def get_observation(self) -> dict:
        self._require_connected()
        try:
            header, payload = self._client.get_observation()
        except BridgeError as exc:
            raise RuntimeError(f'failed to read NERO observation: {exc}') from exc
        state = header.get('state')
        if not isinstance(state, list) or len(state) != len(STATE_NAMES):
            raise RuntimeError(f'bridge returned invalid state: {state!r}')
        image_info = header.get('image')
        if not isinstance(image_info, dict):
            raise RuntimeError('bridge observation has no image metadata')
        observation = {name: float(value) for name, value in zip(STATE_NAMES, state)}
        observation[self._camera_key] = self._decode_image(payload)
        return observation

    def send_action(self, action: dict) -> dict:
        self._require_connected()
        missing = [name for name in STATE_NAMES if name not in action]
        if missing:
            raise ValueError(f'action is missing fields: {missing}')
        values = [float(action[name]) for name in STATE_NAMES]
        if not all(np.isfinite(value) for value in values):
            raise ValueError('action contains a non-finite value')
        try:
            header = self._client.send_action(values)
        except BridgeError as exc:
            raise RuntimeError(f'failed to send NERO action: {exc}') from exc
        if header.get('accepted') is False:
            logger.warning('NERO bridge rejected action: %s', header.get('error', 'unknown error'))
        applied = header.get('action')
        if not isinstance(applied, list) or len(applied) != len(STATE_NAMES):
            raise RuntimeError(f'bridge returned invalid applied action: {applied!r}')
        return {name: float(value) for name, value in zip(STATE_NAMES, applied)}

    def disconnect(self) -> None:
        if not self._connected:
            self._client.close()
            return
        if self.config.stop_on_disconnect:
            try:
                self._client.stop()
            except BridgeError as exc:
                logger.warning('Failed to stop NERO bridge during disconnect: %s', exc)
        self._client.close()
        self._connected = False
        logger.info('Disconnected from NERO bridge')

