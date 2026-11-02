from launch import LaunchDescription
from launch_ros.actions import Node


LEFT_SERIAL = '260522270991'
RIGHT_SERIAL = '261122272091'
SCENE_SERIAL = '263422075415'


def _camera_node(camera_name: str, serial_no: str,
                 color_profile: str, depth_profile: str) -> Node:
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
            'depth_module.depth_profile': depth_profile,
            'depth_module.color_profile': depth_profile,
            'rgb_camera.color_profile': color_profile,
            'align_depth.enable': True,
            'pointcloud.enable': False,
            'wait_for_device_timeout': 10.0,
            'reconnect_timeout': 3.0,
        }],
    )


def generate_launch_description():
    return LaunchDescription([
        _camera_node(
            'camera_scene', SCENE_SERIAL,
            color_profile='640,480,15',
            depth_profile='480,270,15'),
        _camera_node(
            'camera_left_wrist', LEFT_SERIAL,
            color_profile='480,270,15',
            depth_profile='480,270,15'),
        _camera_node(
            'camera_right_wrist', RIGHT_SERIAL,
            color_profile='480,270,15',
            depth_profile='480,270,15'),
    ])
