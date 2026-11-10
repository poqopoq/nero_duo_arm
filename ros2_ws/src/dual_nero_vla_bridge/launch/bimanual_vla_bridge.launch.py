from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'socket_path', default_value='/tmp/nero_bimanual_vla_bridge.sock'),
        DeclareLaunchArgument('action_schema', default_value='rot6d20'),
        DeclareLaunchArgument('execution_mode', default_value='dry_run'),
        DeclareLaunchArgument('allow_real_motion', default_value='false'),
        DeclareLaunchArgument(
            'left_control_command_topic',
            default_value='/left/control/joint_states'),
        DeclareLaunchArgument(
            'right_control_command_topic',
            default_value='/right/control/joint_states'),
        DeclareLaunchArgument('stream_command_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('stream_feedback_timeout_sec', default_value='0.75'),
        DeclareLaunchArgument('gripper_enabled', default_value='false'),
        DeclareLaunchArgument('allow_gripper_motion', default_value='false'),
        DeclareLaunchArgument(
            'left_gripper_width_topic', default_value='/left_gripper/width'),
        DeclareLaunchArgument(
            'right_gripper_width_topic', default_value='/right_gripper/width'),
        DeclareLaunchArgument(
            'left_gripper_command_topic', default_value='/left_gripper/position_cmd'),
        DeclareLaunchArgument(
            'right_gripper_command_topic', default_value='/right_gripper/position_cmd'),
        DeclareLaunchArgument('gripper_max_m', default_value='0.09'),
        DeclareLaunchArgument(
            'camera_scene_topic', default_value='/camera_scene/color/image_raw'),
        DeclareLaunchArgument(
            'camera_scene_compressed_topic',
            default_value='/camera_scene/color/image_raw/compressed'),
        DeclareLaunchArgument('camera_scene_key', default_value='side'),
        DeclareLaunchArgument(
            'camera_left_wrist_topic',
            default_value='/camera_left_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'camera_left_wrist_compressed_topic',
            default_value='/camera_left_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument('camera_left_wrist_key', default_value='left_wrist'),
        DeclareLaunchArgument(
            'camera_right_wrist_topic',
            default_value='/camera_right_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'camera_right_wrist_compressed_topic',
            default_value='/camera_right_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument('camera_right_wrist_key', default_value='right_wrist'),
        DeclareLaunchArgument('use_compressed_color', default_value='true'),
        DeclareLaunchArgument('max_translation_step_m', default_value='0.01'),
        DeclareLaunchArgument('max_rotation_step_rad', default_value='0.05'),
        DeclareLaunchArgument('max_joint_step_rad', default_value='0.15'),
        DeclareLaunchArgument('ik_service', default_value='/compute_ik'),
        DeclareLaunchArgument('left_ik_group_name', default_value='left_arm'),
        DeclareLaunchArgument('right_ik_group_name', default_value='right_arm'),
        DeclareLaunchArgument('left_ik_link_name', default_value='left_tcp_link'),
        DeclareLaunchArgument('right_ik_link_name', default_value='right_tcp_link'),
        DeclareLaunchArgument('left_ik_frame_id', default_value='left_base_link'),
        DeclareLaunchArgument('right_ik_frame_id', default_value='right_base_link'),
        DeclareLaunchArgument('ik_timeout_sec', default_value='0.1'),
        DeclareLaunchArgument('avoid_collisions', default_value='true'),
        DeclareLaunchArgument(
            'trajectory_action',
            default_value='/dual_arm_controller/follow_joint_trajectory'),
        DeclareLaunchArgument('trajectory_duration_sec', default_value='0.2'),
        Node(
            package='dual_nero_vla_bridge',
            executable='bimanual_vla_bridge',
            name='dual_nero_bimanual_vla_bridge',
            output='screen',
            parameters=[{
                'socket_path': LaunchConfiguration('socket_path'),
                'action_schema': LaunchConfiguration('action_schema'),
                'execution_mode': LaunchConfiguration('execution_mode'),
                'allow_real_motion': LaunchConfiguration('allow_real_motion'),
                'left_control_command_topic': LaunchConfiguration(
                    'left_control_command_topic'),
                'right_control_command_topic': LaunchConfiguration(
                    'right_control_command_topic'),
                'stream_command_rate_hz': LaunchConfiguration(
                    'stream_command_rate_hz'),
                'stream_feedback_timeout_sec': LaunchConfiguration(
                    'stream_feedback_timeout_sec'),
                'gripper_enabled': LaunchConfiguration('gripper_enabled'),
                'allow_gripper_motion': LaunchConfiguration(
                    'allow_gripper_motion'),
                'left_gripper_width_topic': LaunchConfiguration(
                    'left_gripper_width_topic'),
                'right_gripper_width_topic': LaunchConfiguration(
                    'right_gripper_width_topic'),
                'left_gripper_command_topic': LaunchConfiguration(
                    'left_gripper_command_topic'),
                'right_gripper_command_topic': LaunchConfiguration(
                    'right_gripper_command_topic'),
                'gripper_max_m': LaunchConfiguration('gripper_max_m'),
                'camera_scene_topic': LaunchConfiguration('camera_scene_topic'),
                'camera_scene_compressed_topic': LaunchConfiguration(
                    'camera_scene_compressed_topic'),
                'camera_scene_key': LaunchConfiguration('camera_scene_key'),
                'camera_left_wrist_topic': LaunchConfiguration(
                    'camera_left_wrist_topic'),
                'camera_left_wrist_compressed_topic': LaunchConfiguration(
                    'camera_left_wrist_compressed_topic'),
                'camera_left_wrist_key': LaunchConfiguration(
                    'camera_left_wrist_key'),
                'camera_right_wrist_topic': LaunchConfiguration(
                    'camera_right_wrist_topic'),
                'camera_right_wrist_compressed_topic': LaunchConfiguration(
                    'camera_right_wrist_compressed_topic'),
                'camera_right_wrist_key': LaunchConfiguration(
                    'camera_right_wrist_key'),
                'use_compressed_color': LaunchConfiguration(
                    'use_compressed_color'),
                'max_translation_step_m': LaunchConfiguration(
                    'max_translation_step_m'),
                'max_rotation_step_rad': LaunchConfiguration(
                    'max_rotation_step_rad'),
                'max_joint_step_rad': LaunchConfiguration('max_joint_step_rad'),
                'ik_service': LaunchConfiguration('ik_service'),
                'left_ik_group_name': LaunchConfiguration('left_ik_group_name'),
                'right_ik_group_name': LaunchConfiguration('right_ik_group_name'),
                'left_ik_link_name': LaunchConfiguration('left_ik_link_name'),
                'right_ik_link_name': LaunchConfiguration('right_ik_link_name'),
                'left_ik_frame_id': LaunchConfiguration('left_ik_frame_id'),
                'right_ik_frame_id': LaunchConfiguration('right_ik_frame_id'),
                'ik_timeout_sec': LaunchConfiguration('ik_timeout_sec'),
                'avoid_collisions': LaunchConfiguration('avoid_collisions'),
                'trajectory_action': LaunchConfiguration('trajectory_action'),
                'trajectory_duration_sec': LaunchConfiguration(
                    'trajectory_duration_sec'),
                'observation_timeout_sec': 0.75,
                'image_timeout_sec': 1.5,
                'workspace_min_xyz': [-2.0, -2.0, -0.2],
                'workspace_max_xyz': [2.0, 2.0, 2.0],
            }],
        ),
    ])
