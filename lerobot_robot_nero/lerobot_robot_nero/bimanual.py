"""LeRobot Robot implementation backed by the bimanual NERO ROS bridge."""
from __future__ import annotations

import logging

import cv2
import numpy as np
from lerobot.robots import Robot

from .bridge_client import BridgeClient, BridgeError
from .config_bimanual import NeroBimanualConfig

logger = logging.getLogger(__name__)


def _pose_names(side: str, schema: str) -> tuple[str, ...]:
    rotation = (
        ('r1', 'r2', 'r3', 'r4', 'r5', 'r6')
        if schema == 'rot6d20'
        else ('rx', 'ry', 'rz')
    )
    return tuple([
        *(f'{side}_tcp.{axis}.pos' for axis in ('x', 'y', 'z')),
        *(f'{side}_tcp.{axis}.pos' for axis in rotation),
        f'{side}_gripper.pos',
    ])


def state_names(schema: str) -> tuple[str, ...]:
    """Return the 20-D rot6d or 14-D axis-angle action ordering."""
    return (*_pose_names('left', schema), *_pose_names('right', schema))


class NeroBimanual(Robot):
    """A bimanual absolute end-effector robot using two NERO arms.

    The plugin talks to ``dual_nero_bimanual_vla_bridge`` over a local Unix
    socket and never imports ``rclpy``.  All action feature names end in
    ``.pos`` so the stock LeRobot 0.6 ``lerobot-rollout`` pipeline routes every
    dimension into the policy action tensor.
    """

    config_class = NeroBimanualConfig
    name = 'nero_bimanual_ros'
    supports_position_hold = True

    def __init__(self, config: NeroBimanualConfig) -> None:
        super().__init__(config)
        self.config = config
        self.robot_type = config.policy_robot_type
        self._client = BridgeClient(config.bridge_socket, config.request_timeout_s)
        self._connected = False
        self._names = state_names(config.action_schema)
        self._camera_sizes = {
            config.scene_camera_key: (
                config.scene_image_height, config.scene_image_width),
            config.left_wrist_camera_key: (
                config.left_wrist_image_height, config.left_wrist_image_width),
            config.right_wrist_camera_key: (
                config.right_wrist_image_height, config.right_wrist_image_width),
        }

    @property
    def observation_features(self) -> dict:
        features: dict = {name: float for name in self._names}
        for key, shape in self._camera_sizes.items():
            features[key] = (*shape, 3)
        return features

    @property
    def action_features(self) -> dict:
        return {name: float for name in self._names}

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
        if not status.get('bimanual'):
            self._client.close()
            raise ConnectionError('bridge is not a bimanual NERO bridge')
        if status.get('schema') != self.config.action_schema:
            self._client.close()
            raise ConnectionError(
                f'bridge schema is {status.get("schema")!r}, '
                f'expected {self.config.action_schema!r}')
        bridge_keys = set(status.get('camera_keys') or [])
        expected_keys = set(self._camera_sizes)
        if bridge_keys != expected_keys:
            self._client.close()
            raise ConnectionError(
                f'bridge camera keys are {sorted(bridge_keys)}, '
                f'expected {sorted(expected_keys)}')
        if calibrate:
            self.calibrate()
        self._connected = True
        logger.info(
            'Connected to bimanual NERO bridge at %s (%s)',
            self.config.bridge_socket,
            self.config.action_schema,
        )

    def _require_connected(self) -> None:
        if not self._connected:
            raise RuntimeError('NeroBimanual robot is not connected')

    def _decode_image(self, payload: bytes, size: tuple[int, int]) -> np.ndarray:
        image = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError('bridge returned an image that could not be decoded')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if image.shape[:2] != size:
            image = cv2.resize(
                image, (size[1], size[0]), interpolation=cv2.INTER_AREA)
        return np.ascontiguousarray(image.astype(np.uint8, copy=False))

    def get_observation(self) -> dict:
        self._require_connected()
        try:
            header, payload = self._client.get_observation()
        except BridgeError as exc:
            raise RuntimeError(
                f'failed to read bimanual NERO observation: {exc}') from exc
        state = header.get('state')
        if not isinstance(state, list) or len(state) != len(self._names):
            raise RuntimeError(f'bridge returned invalid state: {state!r}')
        observation = {
            name: float(value) for name, value in zip(self._names, state)
        }
        image_metadata = header.get('images')
        if not isinstance(image_metadata, list):
            raise RuntimeError('bridge observation has no image metadata')
        decoded: dict[str, np.ndarray] = {}
        for item in image_metadata:
            key = item.get('key')
            if key not in self._camera_sizes:
                raise RuntimeError(f'bridge returned unexpected image key: {key!r}')
            offset = int(item.get('offset', -1))
            size = int(item.get('size', -1))
            if offset < 0 or size < 0 or offset + size > len(payload):
                raise RuntimeError(f'invalid image payload range for {key!r}')
            decoded[key] = self._decode_image(
                payload[offset:offset + size], self._camera_sizes[key])
        missing = set(self._camera_sizes) - set(decoded)
        if missing:
            raise RuntimeError(f'bridge observation is missing images: {sorted(missing)}')
        observation.update(decoded)
        return observation

    def send_action(self, action: dict) -> dict:
        self._require_connected()
        missing = [name for name in self._names if name not in action]
        if missing:
            raise ValueError(f'action is missing fields: {missing}')
        values = [float(action[name]) for name in self._names]
        if not all(np.isfinite(value) for value in values):
            raise ValueError('action contains a non-finite value')
        try:
            header = self._client.send_action(values)
        except BridgeError as exc:
            raise RuntimeError(f'failed to send bimanual NERO action: {exc}') from exc
        if header.get('accepted') is False:
            error = header.get('error', 'unknown error')
            logger.error('Bimanual NERO bridge rejected action: %s', error)
            raise RuntimeError(f'bimanual NERO bridge rejected action: {error}')
        applied = header.get('action')
        if not isinstance(applied, list) or len(applied) != len(self._names):
            raise RuntimeError(
                f'bridge returned invalid applied action: {applied!r}')
        return {name: float(value) for name, value in zip(self._names, applied)}

    def disconnect(self) -> None:
        if not self._connected:
            self._client.close()
            return
        if self.config.stop_on_disconnect:
            try:
                self._client.stop()
            except BridgeError as exc:
                logger.warning(
                    'Failed to stop bimanual NERO bridge during disconnect: %s', exc)
        self._client.close()
        self._connected = False
        logger.info('Disconnected from bimanual NERO bridge')
