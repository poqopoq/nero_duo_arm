from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots import RobotConfig


ACTION_SCHEMAS = ('rot6d20', 'axis_angle14')


@RobotConfig.register_subclass('nero_bimanual_ros')
@dataclass
class NeroBimanualConfig(RobotConfig):
    """Configuration for the bimanual NERO ROS bridge client.

    ``action_schema`` must match the checkpoint's training data:

    - ``rot6d20``: Xense UMI layout, two 10-D blocks with rotation-matrix
      first two columns.
    - ``axis_angle14``: two 7-D absolute axis-angle blocks.

    The camera keys default to the Xense LeRobot dataset names.  Change them
    when the checkpoint uses different ``observation.images.*`` names.
    """

    bridge_socket: str = '/tmp/nero_bimanual_vla_bridge.sock'
    action_schema: str = 'rot6d20'
    policy_robot_type: str = 'bi_rdt_gripper'

    scene_camera_key: str = 'side'
    left_wrist_camera_key: str = 'left_wrist'
    right_wrist_camera_key: str = 'right_wrist'
    scene_image_width: int = 480
    scene_image_height: int = 480
    left_wrist_image_width: int = 480
    left_wrist_image_height: int = 480
    right_wrist_image_width: int = 480
    right_wrist_image_height: int = 480

    request_timeout_s: float = 2.0
    stop_on_disconnect: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.action_schema not in ACTION_SCHEMAS:
            raise ValueError(
                f'action_schema must be one of {ACTION_SCHEMAS}, '
                f'got {self.action_schema!r}')
        camera_keys = (
            self.scene_camera_key,
            self.left_wrist_camera_key,
            self.right_wrist_camera_key,
        )
        if any(not key for key in camera_keys):
            raise ValueError('camera keys must not be empty')
        if len(set(camera_keys)) != len(camera_keys):
            raise ValueError('camera keys must be unique')
        if min(
            self.scene_image_width,
            self.scene_image_height,
            self.left_wrist_image_width,
            self.left_wrist_image_height,
            self.right_wrist_image_width,
            self.right_wrist_image_height,
        ) <= 0:
            raise ValueError('camera image dimensions must be positive')
        if self.request_timeout_s <= 0:
            raise ValueError('request_timeout_s must be positive')
