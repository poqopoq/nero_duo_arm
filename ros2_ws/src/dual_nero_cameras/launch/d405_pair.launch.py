from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LEFT_SERIAL = '260522270991'
RIGHT_SERIAL = '261122272091'
PROFILE = '640,480,30'


def _camera_node(camera_name: str, serial_no: str, enable_depth) -> Node:
    return Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='/',
        name=camera_name,
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'serial_no': serial_no,
            'enable_color': True,
            'enable_depth': enable_depth,
            'enable_sync': False,
            'enable_infra': False,
            'enable_infra1': False,
            'enable_infra2': False,
            'depth_module.depth_profile': PROFILE,
            'depth_module.color_profile': PROFILE,
            'rgb_camera.color_profile': PROFILE,
            'rgb_camera.color_format': 'RGB8',
            'align_depth.enable': enable_depth,
            'pointcloud.enable': False,
            'wait_for_device_timeout': 10.0,
            'reconnect_timeout': 3.0,
        }],
    )


def generate_launch_description():
    enable_depth = LaunchConfiguration('enable_depth')
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_depth', default_value='false',
            description='Enable D405 depth. RGB-only is the default data path.'),
        _camera_node('camera_left_wrist', LEFT_SERIAL, enable_depth),
        _camera_node('camera_right_wrist', RIGHT_SERIAL, enable_depth),
    ])
