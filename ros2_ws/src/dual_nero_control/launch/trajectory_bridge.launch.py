from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument(
            'cancel_service_name',
            default_value='/dual_nero_trajectory_bridge/cancel_motion'),
        DeclareLaunchArgument(
            'clear_cancel_service_name',
            default_value='/dual_nero_trajectory_bridge/clear_cancel'),
        Node(
            package='dual_nero_control',
            executable='trajectory_bridge',
            name='dual_nero_trajectory_bridge',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
                'command_rate_hz': 50.0,
                'startup_feedback_timeout': 5.0,
                'final_hold_sec': 0.5,
                'cancel_service_name': LaunchConfiguration('cancel_service_name'),
                'clear_cancel_service_name': LaunchConfiguration(
                    'clear_cancel_service_name'),
            }],
        ),
    ])
