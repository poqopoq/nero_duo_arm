from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('socket_path', default_value='/tmp/nero_vla_bridge.sock'),
        DeclareLaunchArgument('side', default_value='left'),
        DeclareLaunchArgument('image_topic', default_value='/camera_scene/color/image_raw'),
        DeclareLaunchArgument(
            'image_compressed_topic',
            default_value='/camera_scene/color/image_raw/compressed'),
        DeclareLaunchArgument('use_compressed_image', default_value='true'),
        DeclareLaunchArgument('image_key', default_value='camera1'),
        DeclareLaunchArgument('execution_mode', default_value='dry_run'),
        DeclareLaunchArgument('allow_real_motion', default_value='false'),
        DeclareLaunchArgument('control_command_topic', default_value=''),
        DeclareLaunchArgument('stream_command_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('stream_feedback_timeout_sec', default_value='0.75'),
        DeclareLaunchArgument('gripper_enabled', default_value='false'),
        DeclareLaunchArgument('allow_gripper_motion', default_value='false'),
        DeclareLaunchArgument('gripper_width_topic', default_value=''),
        DeclareLaunchArgument('gripper_command_topic', default_value=''),
        DeclareLaunchArgument('gripper_max_m', default_value='0.09'),
        DeclareLaunchArgument('max_translation_step_m', default_value='0.01'),
        DeclareLaunchArgument('max_rotation_step_rad', default_value='0.05'),
        DeclareLaunchArgument('max_joint_step_rad', default_value='0.15'),
        DeclareLaunchArgument('ik_service', default_value='/compute_ik'),
        DeclareLaunchArgument('ik_group_name', default_value=''),
        DeclareLaunchArgument('ik_link_name', default_value=''),
        DeclareLaunchArgument('ik_frame_id', default_value=''),
        DeclareLaunchArgument('ik_timeout_sec', default_value='0.1'),
        DeclareLaunchArgument('avoid_collisions', default_value='true'),
        DeclareLaunchArgument(
            'trajectory_action',
            default_value='/dual_arm_controller/follow_joint_trajectory'),
        DeclareLaunchArgument('trajectory_duration_sec', default_value='0.2'),
        Node(
            package='dual_nero_vla_bridge',
            executable='vla_bridge',
            name='dual_nero_vla_bridge',
            output='screen',
            parameters=[{
                'socket_path': LaunchConfiguration('socket_path'),
                'side': LaunchConfiguration('side'),
                'image_topic': LaunchConfiguration('image_topic'),
                'image_compressed_topic': LaunchConfiguration(
                    'image_compressed_topic'),
                'use_compressed_image': LaunchConfiguration(
                    'use_compressed_image'),
                'image_key': LaunchConfiguration('image_key'),
                'execution_mode': LaunchConfiguration('execution_mode'),
                'allow_real_motion': LaunchConfiguration('allow_real_motion'),
                'control_command_topic': LaunchConfiguration(
                    'control_command_topic'),
                'stream_command_rate_hz': LaunchConfiguration(
                    'stream_command_rate_hz'),
                'stream_feedback_timeout_sec': LaunchConfiguration(
                    'stream_feedback_timeout_sec'),
                'gripper_enabled': LaunchConfiguration('gripper_enabled'),
                'allow_gripper_motion': LaunchConfiguration(
                    'allow_gripper_motion'),
                'gripper_width_topic': LaunchConfiguration('gripper_width_topic'),
                'gripper_command_topic': LaunchConfiguration(
                    'gripper_command_topic'),
                'gripper_max_m': LaunchConfiguration('gripper_max_m'),
                'max_translation_step_m': LaunchConfiguration('max_translation_step_m'),
                'max_rotation_step_rad': LaunchConfiguration('max_rotation_step_rad'),
                'max_joint_step_rad': LaunchConfiguration('max_joint_step_rad'),
                'ik_service': LaunchConfiguration('ik_service'),
                'ik_group_name': LaunchConfiguration('ik_group_name'),
                'ik_link_name': LaunchConfiguration('ik_link_name'),
                'ik_frame_id': LaunchConfiguration('ik_frame_id'),
                'ik_timeout_sec': LaunchConfiguration('ik_timeout_sec'),
                'avoid_collisions': LaunchConfiguration('avoid_collisions'),
                'trajectory_action': LaunchConfiguration('trajectory_action'),
                'trajectory_duration_sec': LaunchConfiguration('trajectory_duration_sec'),
                'observation_timeout_sec': 0.75,
                'image_timeout_sec': 1.5,
                'workspace_min_xyz': [-2.0, -2.0, -0.2],
                'workspace_max_xyz': [2.0, 2.0, 2.0],
            }],
        ),
    ])

