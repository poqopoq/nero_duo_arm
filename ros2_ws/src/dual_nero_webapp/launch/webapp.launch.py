import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('dual_nero_control')
    cameras_share = get_package_share_directory('dual_nero_cameras')
    description_share = get_package_share_directory('dual_nero_description')
    moveit_share = get_package_share_directory('dual_nero_moveit_config')
    webapp_share = get_package_share_directory('dual_nero_webapp')
    xacro_path = os.path.join(
        description_share, 'urdf', 'two_nero_tabletop.urdf.xacro')
    support_pose_path = os.path.join(
        webapp_share, 'config', 'support_pose.yaml')

    start_drivers = LaunchConfiguration('start_drivers')
    start_cameras = LaunchConfiguration('start_cameras')
    start_moveit = LaunchConfiguration('start_moveit')
    moveit_dry_run = LaunchConfiguration('moveit_dry_run')
    moveit_rviz = LaunchConfiguration('moveit_rviz')
    enable_cameras = LaunchConfiguration('enable_cameras')

    webapp = Node(
        package='dual_nero_webapp',
        executable='webapp_node',
        name='dual_nero_webapp',
        output='screen',
        parameters=[{
            'left_ns': LaunchConfiguration('left_ns'),
            'right_ns': LaunchConfiguration('right_ns'),
            'http_host': LaunchConfiguration('http_host'),
            'http_port': LaunchConfiguration('http_port'),
            'initial_gate_open': LaunchConfiguration('initial_gate_open'),
            'enable_cameras': enable_cameras,
            'depth_min_m': LaunchConfiguration('depth_min_m'),
            'depth_max_m': LaunchConfiguration('depth_max_m'),
            'use_compressed_color': LaunchConfiguration('use_compressed_color'),
            'scene_color_topic': LaunchConfiguration('scene_color_topic'),
            'scene_color_compressed_topic': LaunchConfiguration(
                'scene_color_compressed_topic'),
            'scene_depth_topic': LaunchConfiguration('scene_depth_topic'),
            'left_wrist_color_topic': LaunchConfiguration('left_wrist_color_topic'),
            'left_wrist_color_compressed_topic': LaunchConfiguration(
                'left_wrist_color_compressed_topic'),
            'left_wrist_depth_topic': LaunchConfiguration('left_wrist_depth_topic'),
            'right_wrist_color_topic': LaunchConfiguration('right_wrist_color_topic'),
            'right_wrist_color_compressed_topic': LaunchConfiguration(
                'right_wrist_color_compressed_topic'),
            'right_wrist_depth_topic': LaunchConfiguration('right_wrist_depth_topic'),
            'support_pose_file': support_pose_path,
            'urdf_xacro': xacro_path,
            'trajectory_cancel_service': LaunchConfiguration(
                'trajectory_cancel_service'),
            'trajectory_clear_cancel_service': LaunchConfiguration(
                'trajectory_clear_cancel_service'),
            'moveit_dry_run': moveit_dry_run,
            'table_width': LaunchConfiguration('table_width'),
            'table_depth': LaunchConfiguration('table_depth'),
            'table_thickness': LaunchConfiguration('table_thickness'),
            'base_spacing': LaunchConfiguration('base_spacing'),
            'front_edge_offset': LaunchConfiguration('front_edge_offset'),
            'left_base_yaw': LaunchConfiguration('left_base_yaw'),
            'right_base_yaw': LaunchConfiguration('right_base_yaw'),
            'include_gripper': LaunchConfiguration('include_gripper'),
        }],
    )

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'dual_nero_drivers.launch.py')),
        condition=IfCondition(start_drivers),
        launch_arguments={
            'can_left': LaunchConfiguration('can_left'),
            'can_right': LaunchConfiguration('can_right'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'auto_enable': LaunchConfiguration('auto_enable'),
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cameras_share, 'launch', 'all_cameras.launch.py')),
        condition=IfCondition(start_cameras),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share, 'launch', 'moveit_execution.launch.py')),
        condition=IfCondition(start_moveit),
        launch_arguments={
            'dry_run': moveit_dry_run,
            'rviz': moveit_rviz,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_ns', default_value='left',
            description='ROS namespace for the left arm driver.'),
        DeclareLaunchArgument(
            'right_ns', default_value='right',
            description='ROS namespace for the right arm driver.'),
        DeclareLaunchArgument(
            'http_host', default_value='0.0.0.0',
            description='HTTP bind address. Use 127.0.0.1 to keep the UI local.'),
        DeclareLaunchArgument(
            'http_port', default_value='8080', description='HTTP port.'),
        DeclareLaunchArgument(
            'initial_gate_open', default_value='false',
            description='Only controls the webapp-side interlock; the driver '
                        'control gate still starts closed.'),
        DeclareLaunchArgument(
            'start_drivers', default_value='false',
            description='Also launch the two namespaced NERO drivers.'),
        DeclareLaunchArgument(
            'start_cameras', default_value='false',
            description='Also launch the three RealSense camera nodes.'),
        DeclareLaunchArgument(
            'start_moveit', default_value='false',
            description='Also launch MoveIt move_group and the trajectory bridge. '
                        'The webapp uses /move_action for all motions.'),
        DeclareLaunchArgument(
            'moveit_dry_run', default_value='true',
            description='When starting MoveIt, simulate execution without '
                        'commanding the drivers. Set false only for validated '
                        'real-hardware motion.'),
        DeclareLaunchArgument(
            'moveit_rviz', default_value='false',
            description='Open RViz when start_moveit is true.'),
        DeclareLaunchArgument(
            'trajectory_cancel_service',
            default_value='/dual_nero_trajectory_bridge/cancel_motion',
            description='Service used to stop the active trajectory bridge goal.'),
        DeclareLaunchArgument(
            'trajectory_clear_cancel_service',
            default_value='/dual_nero_trajectory_bridge/clear_cancel',
            description='Service used to clear the bridge cancellation latch.'),
        DeclareLaunchArgument(
            'enable_cameras', default_value='true',
            description='Subscribe to the configured ROS camera topics.'),
        DeclareLaunchArgument('depth_min_m', default_value='0.05'),
        DeclareLaunchArgument('depth_max_m', default_value='2.0'),
        DeclareLaunchArgument(
            'use_compressed_color', default_value='true',
            description='Subscribe to relayed JPEG color topics for stable 30 FPS.'),
        DeclareLaunchArgument(
            'scene_color_compressed_topic',
            default_value='/camera_scene/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'scene_color_topic', default_value='/camera_scene/color/image_raw'),
        DeclareLaunchArgument(
            'scene_depth_topic',
            default_value='/camera_scene/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'left_wrist_color_topic',
            default_value='/camera_left_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'left_wrist_color_compressed_topic',
            default_value='/camera_left_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'left_wrist_depth_topic',
            default_value='/camera_left_wrist/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_topic',
            default_value='/camera_right_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_compressed_topic',
            default_value='/camera_right_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'right_wrist_depth_topic',
            default_value='/camera_right_wrist/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('can_left', default_value='can_left'),
        DeclareLaunchArgument('can_right', default_value='can_right'),
        DeclareLaunchArgument('speed_percent', default_value='5'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        DeclareLaunchArgument('table_width', default_value='1.80'),
        DeclareLaunchArgument('table_depth', default_value='0.90'),
        DeclareLaunchArgument('table_thickness', default_value='0.03'),
        DeclareLaunchArgument('base_spacing', default_value='0.70'),
        DeclareLaunchArgument('front_edge_offset', default_value='0.06'),
        DeclareLaunchArgument('left_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('right_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('include_gripper', default_value='false'),
        webapp,
        drivers,
        cameras,
        moveit,
    ])
