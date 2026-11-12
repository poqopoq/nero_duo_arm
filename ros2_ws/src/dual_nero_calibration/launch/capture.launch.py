from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera', default_value='left_wrist'),
        DeclareLaunchArgument('samples', default_value='20'),
        DeclareLaunchArgument('output_dir', default_value=''),
        DeclareLaunchArgument('max_reprojection_rms_px', default_value='1.0'),
        DeclareLaunchArgument('min_corner_gap_px', default_value='2.0'),
        Node(
            package='dual_nero_calibration',
            executable='capture_chessboard',
            name='capture_chessboard',
            output='screen',
            parameters=[{
                'camera': LaunchConfiguration('camera'),
                'samples': LaunchConfiguration('samples'),
                'output_dir': LaunchConfiguration('output_dir'),
                'max_reprojection_rms_px': LaunchConfiguration(
                    'max_reprojection_rms_px'),
                'min_corner_gap_px': LaunchConfiguration('min_corner_gap_px'),
            }],
        ),
    ])
