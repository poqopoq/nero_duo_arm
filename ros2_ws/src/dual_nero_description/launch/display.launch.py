from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('dual_nero_description')
    model = PathJoinSubstitution([share, 'urdf', 'two_nero_tabletop.urdf.xacro'])
    rviz_config = PathJoinSubstitution([share, 'rviz', 'dual_nero.rviz'])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', model,
            ' table_width:=', LaunchConfiguration('table_width'),
            ' table_depth:=', LaunchConfiguration('table_depth'),
            ' base_spacing:=', LaunchConfiguration('base_spacing'),
            ' front_edge_offset:=', LaunchConfiguration('front_edge_offset'),
            ' left_base_yaw:=', LaunchConfiguration('left_base_yaw'),
            ' right_base_yaw:=', LaunchConfiguration('right_base_yaw'),
            ' include_gripper:=', LaunchConfiguration('include_gripper'),
            ' left_tcp_x:=', LaunchConfiguration('left_tcp_x'),
            ' left_tcp_y:=', LaunchConfiguration('left_tcp_y'),
            ' left_tcp_z:=', LaunchConfiguration('left_tcp_z'),
            ' left_tcp_roll:=', LaunchConfiguration('left_tcp_roll'),
            ' left_tcp_pitch:=', LaunchConfiguration('left_tcp_pitch'),
            ' left_tcp_yaw:=', LaunchConfiguration('left_tcp_yaw'),
            ' right_tcp_x:=', LaunchConfiguration('right_tcp_x'),
            ' right_tcp_y:=', LaunchConfiguration('right_tcp_y'),
            ' right_tcp_z:=', LaunchConfiguration('right_tcp_z'),
            ' right_tcp_roll:=', LaunchConfiguration('right_tcp_roll'),
            ' right_tcp_pitch:=', LaunchConfiguration('right_tcp_pitch'),
            ' right_tcp_yaw:=', LaunchConfiguration('right_tcp_yaw'),
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('table_width', default_value='1.80'),
        DeclareLaunchArgument('table_depth', default_value='0.90'),
        DeclareLaunchArgument('base_spacing', default_value='0.60'),
        DeclareLaunchArgument('front_edge_offset', default_value='0.06'),
        DeclareLaunchArgument('left_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('right_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('include_gripper', default_value='true'),
        DeclareLaunchArgument('left_tcp_x', default_value='0.0'),
        DeclareLaunchArgument('left_tcp_y', default_value='0.0'),
        DeclareLaunchArgument('left_tcp_z', default_value='0.1708'),
        DeclareLaunchArgument('left_tcp_roll', default_value='0.0'),
        DeclareLaunchArgument('left_tcp_pitch', default_value='0.0'),
        DeclareLaunchArgument('left_tcp_yaw', default_value='0.0'),
        DeclareLaunchArgument('right_tcp_x', default_value='0.0'),
        DeclareLaunchArgument('right_tcp_y', default_value='0.0'),
        DeclareLaunchArgument('right_tcp_z', default_value='0.1708'),
        DeclareLaunchArgument('right_tcp_roll', default_value='0.0'),
        DeclareLaunchArgument('right_tcp_pitch', default_value='0.0'),
        DeclareLaunchArgument('right_tcp_yaw', default_value='0.0'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('gui')),
        ),
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('rviz')),
                ),
            ],
        ),
    ])
