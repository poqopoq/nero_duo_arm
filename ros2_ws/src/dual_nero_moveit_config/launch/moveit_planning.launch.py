import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_gui = LaunchConfiguration('gui')
    use_rviz = LaunchConfiguration('rviz')
    collision_padding = LaunchConfiguration('collision_padding')

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
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()
    moveit_params['robot_description_planning.default_robot_padding'] = (
        ParameterValue(collision_padding, value_type=float))
    moveit_params['robot_description_planning.default_object_padding'] = (
        ParameterValue(collision_padding, value_type=float))
    moveit_params['robot_description_planning.default_attached_padding'] = (
        ParameterValue(collision_padding, value_type=float))
    for key in ('moveit_controller_manager', 'moveit_simple_controller_manager',
                'trajectory_execution'):
        moveit_params.pop(key, None)

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[moveit_params],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[moveit_config.robot_description],
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(use_gui),
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_gui),
    )

    rviz_config = os.path.join(
        get_package_share_directory('dual_nero_moveit_config'),
        'rviz', 'dual_nero_moveit.rviz')

    rviz = TimerAction(
        period=2.0,
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
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'collision_padding', default_value='0.025',
            description='Extra collision clearance added around robot and scene '
                        'geometry in meters.'),
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        move_group,
        rviz,
    ])
