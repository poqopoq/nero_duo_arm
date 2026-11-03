from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


LEFT_SERIAL = '260522270991'
RIGHT_SERIAL = '261122272091'
SCENE_SERIAL = '263422075415'
CAMERA_FPS = 30
COLOR_PROFILE = f'640,480,{CAMERA_FPS}'
DEPTH_PROFILE = f'640,480,{CAMERA_FPS}'


def _camera_node(camera_name: str, serial_no: str,
                 color_profile: str, depth_profile: str,
                 enable_depth, color_format: str) -> Node:
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
            'depth_module.depth_profile': depth_profile,
            'depth_module.color_profile': depth_profile,
            'depth_module.color_format': color_format,
            'rgb_camera.color_profile': color_profile,
            'rgb_camera.color_format': color_format,
            'align_depth.enable': enable_depth,
            'pointcloud.enable': False,
            'wait_for_device_timeout': 10.0,
            'reconnect_timeout': 3.0,
        }],
    )


def generate_launch_description():
    enable_scene_depth = LaunchConfiguration('enable_scene_depth')
    enable_wrist_depth = LaunchConfiguration('enable_wrist_depth')
    color_relay = Node(
        package='dual_nero_camera_relay',
        executable='color_jpeg_relay',
        name='color_jpeg_relay',
        output='screen',
        parameters=[{
            'scene_input': '/camera_scene/color/image_raw',
            'scene_output': '/camera_scene/color/image_raw/compressed',
            'left_wrist_input': '/camera_left_wrist/color/image_raw',
            'left_wrist_output': '/camera_left_wrist/color/image_raw/compressed',
            'right_wrist_input': '/camera_right_wrist/color/image_raw',
            'right_wrist_output': '/camera_right_wrist/color/image_raw/compressed',
            'jpeg_quality': 78,
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_scene_depth', default_value='false',
            description='Enable Scene depth. Keep false for stable 30 FPS color; '
                        'the D435 drops color frames when Color and Depth run '
                        'together at 30 FPS.'),
        DeclareLaunchArgument(
            'enable_wrist_depth', default_value='false',
            description='Enable the two D405 wrist depth streams at 30 FPS. '
                        'Disabled by default because the LeRobot/VLA path uses '
                        'RGB only.'),
        _camera_node(
            'camera_scene', SCENE_SERIAL,
            color_profile=COLOR_PROFILE,
            depth_profile=DEPTH_PROFILE,
            enable_depth=enable_scene_depth, color_format='YUYV'),
        _camera_node(
            'camera_left_wrist', LEFT_SERIAL,
            color_profile=COLOR_PROFILE,
            depth_profile=DEPTH_PROFILE,
            enable_depth=enable_wrist_depth, color_format='RGB8'),
        _camera_node(
            'camera_right_wrist', RIGHT_SERIAL,
            color_profile=COLOR_PROFILE,
            depth_profile=DEPTH_PROFILE,
            enable_depth=enable_wrist_depth, color_format='RGB8'),
        color_relay,
    ])
