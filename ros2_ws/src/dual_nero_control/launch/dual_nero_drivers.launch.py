from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    can_left = LaunchConfiguration('can_left')
    can_right = LaunchConfiguration('can_right')
    speed_percent = LaunchConfiguration('speed_percent')
    auto_enable = LaunchConfiguration('auto_enable')

    common = {
        'arm_type': 'nero',
        'effector_type': 'none',
        'auto_enable': auto_enable,
        'speed_percent': speed_percent,
        'control_enabled': False,
        'pub_rate': 200,
    }

    return LaunchDescription([
        DeclareLaunchArgument('can_left', default_value='can_left'),
        DeclareLaunchArgument('can_right', default_value='can_right'),
        DeclareLaunchArgument('speed_percent', default_value='5'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        Node(
            package='agx_arm_ctrl',
            executable='agx_arm_ctrl_single',
            namespace='left',
            name='agx_arm_ctrl_single_node',
            output='screen',
            parameters=[dict(common, can_port=can_left)],
        ),
        Node(
            package='agx_arm_ctrl',
            executable='agx_arm_ctrl_single',
            namespace='right',
            name='agx_arm_ctrl_single_node',
            output='screen',
            parameters=[dict(common, can_port=can_right)],
        ),
    ])
