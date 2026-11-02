from launch import LaunchDescription
from launch_ros.actions import Node


LEFT_SERIAL = '260522270991'
RIGHT_SERIAL = '261122272091'
PROFILE = '480,270,15'


def _camera_node(camera_name: str, serial_no: str) -> Node:
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='/',
        name=camera_name,
        output='screen',
        parameters=[{
            'serial_no': serial_no,
            'enable_color': True,
            'enable_depth': True,
            'enable_sync': False,
            'depth_module.depth_profile': PROFILE,
            'depth_module.color_profile': PROFILE,
            'rgb_camera.color_profile': PROFILE,
            'align_depth.enable': True,
            'pointcloud.enable': False,
            'wait_for_device_timeout': 10.0,
            'reconnect_timeout': 3.0,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        _camera_node('camera_left_wrist', LEFT_SERIAL),
        _camera_node('camera_right_wrist', RIGHT_SERIAL),
    ])
