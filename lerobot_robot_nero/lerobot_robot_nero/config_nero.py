from __future__ import annotations

from dataclasses import dataclass

from lerobot.robots import RobotConfig


@RobotConfig.register_subclass('nero_ros')
@dataclass
class NeroConfig(RobotConfig):
    """Configuration for the NERO ROS bridge client.

    The LeRobot process never imports ``rclpy``. It talks to
    ``dual_nero_vla_bridge`` over a local Unix socket instead.

    The feature contract is ``ee.x.pos ... ee.gripper.pos``. Datasets and
    checkpoints created with the older ``ee.x ... ee.gripper`` names must be
    re-exported or renamed before remote rollout.
    """

    bridge_socket: str = '/tmp/nero_vla_bridge.sock'
    side: str = 'left'
    camera_key: str = 'camera1'
    image_width: int = 640
    image_height: int = 480
    image_fps: int = 15
    request_timeout_s: float = 2.0
    stop_on_disconnect: bool = True

    def __post_init__(self) -> None:
        super().__post_init__()
        if self.side not in ('left', 'right'):
            raise ValueError("side must be 'left' or 'right'")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError('image_width and image_height must be positive')
        if self.image_fps <= 0:
            raise ValueError('image_fps must be positive')
        if self.request_timeout_s <= 0:
            raise ValueError('request_timeout_s must be positive')

