from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='left_wrist'),
        DeclareLaunchArgument('samples', default_value='20'),
        DeclareLaunchArgument('output_dir', default_value=''),
        Node(
            package='dual_nero_calibration',
            executable='capture_chessboard',
            name='capture_chessboard',
            output='screen',
            parameters=[{
                'camera': LaunchConfiguration('camera'),
                'samples': LaunchConfiguration('samples'),
                'output_dir': LaunchConfiguration('output_dir'),
            }],
        ),
    ])
