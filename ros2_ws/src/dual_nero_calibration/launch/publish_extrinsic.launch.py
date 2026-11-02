from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('yaml_files', default_value=''),
        Node(
            package='dual_nero_calibration',
            executable='publish_extrinsic',
            name='camera_extrinsic_publisher',
            output='screen',
            parameters=[{'yaml_files': LaunchConfiguration('yaml_files')}],
        ),
    ])
