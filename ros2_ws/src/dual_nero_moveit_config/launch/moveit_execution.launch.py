import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')
    dry_run = LaunchConfiguration('dry_run')

    description_share = get_package_share_directory('dual_nero_description')
    xacro_path = os.path.join(description_share, 'urdf',
                              'two_nero_tabletop.urdf.xacro')

    moveit_config = (
        MoveItConfigsBuilder(
            'dual_nero_tabletop', package_name='dual_nero_moveit_config')
        .robot_description(file_path=xacro_path)
        .robot_description_semantic(file_path='config/dual_nero.srdf')
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .planning_pipelines(pipelines=['ompl'])
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    bridge = Node(
        package='dual_nero_control',
        executable='trajectory_bridge',
        name='dual_nero_trajectory_bridge',
        output='screen',
        parameters=[{
            'dry_run': dry_run,
            'command_rate_hz': 50.0,
            'startup_feedback_timeout': 5.0,
            'final_hold_sec': 0.5,
        }],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen',
    )

    move_group = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='moveit_ros_move_group',
                executable='move_group',
                output='screen',
                parameters=[moveit_config.to_dict()],
            )
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory('dual_nero_moveit_config'),
        'rviz', 'dual_nero_moveit.rviz')
    rviz = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                arguments=['-d', rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.planning_pipelines,
                    moveit_config.robot_description_kinematics,
                ],
                output='screen',
                condition=IfCondition(use_rviz),
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        bridge,
        robot_state_publisher,
        move_group,
        rviz,
    ])
