import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_rviz = LaunchConfiguration('rviz')
    collision_padding = LaunchConfiguration('collision_padding')
    dry_run = LaunchConfiguration('dry_run')
    command_rate_hz = LaunchConfiguration('command_rate_hz')
    trajectory_interpolation = LaunchConfiguration('trajectory_interpolation')
    settle_timeout_sec = LaunchConfiguration('settle_timeout_sec')
    settle_tolerance_rad = LaunchConfiguration('settle_tolerance_rad')
    gripper = LaunchConfiguration('gripper')
    gripper_allow_motion = LaunchConfiguration('gripper_allow_motion')
    gripper_enable_motor = LaunchConfiguration('gripper_enable_motor')
    gripper_port = LaunchConfiguration('gripper_port')
    gripper_can_fd = LaunchConfiguration('gripper_can_fd')
    gripper_brs = LaunchConfiguration('gripper_brs')
    left_gripper = LaunchConfiguration('left_gripper')
    left_gripper_allow_motion = LaunchConfiguration(
        'left_gripper_allow_motion')
    left_gripper_enable_motor = LaunchConfiguration(
        'left_gripper_enable_motor')
    left_gripper_port = LaunchConfiguration('left_gripper_port')
    left_gripper_can_fd = LaunchConfiguration('left_gripper_can_fd')
    left_gripper_brs = LaunchConfiguration('left_gripper_brs')
    gripper_speed_percent = LaunchConfiguration('gripper_speed_percent')
    gripper_torque_percent = LaunchConfiguration('gripper_torque_percent')
    gripper_ramp_ms = LaunchConfiguration('gripper_ramp_ms')
    gripper_max_current = LaunchConfiguration('gripper_max_current')
    gripper_position_tolerance = LaunchConfiguration(
        'gripper_position_tolerance')
    gripper_command_max = LaunchConfiguration('gripper_command_max')

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

    moveit_params = moveit_config.to_dict()
    moveit_params['robot_description_planning.default_robot_padding'] = (
        ParameterValue(collision_padding, value_type=float))
    moveit_params['robot_description_planning.default_object_padding'] = (
        ParameterValue(collision_padding, value_type=float))
    moveit_params['robot_description_planning.default_attached_padding'] = (
        ParameterValue(collision_padding, value_type=float))

    bridge = Node(
        package='dual_nero_control',
        executable='trajectory_bridge',
        name='dual_nero_trajectory_bridge',
        output='screen',
        parameters=[{
            'dry_run': dry_run,
            'command_rate_hz': command_rate_hz,
            'interpolation': trajectory_interpolation,
            'settle_timeout_sec': settle_timeout_sec,
            'settle_tolerance_rad': settle_tolerance_rad,
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
                parameters=[moveit_params],
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

    gripper_node = Node(
        package='dual_nero_control',
        executable='right_gripper_node',
        name='right_gripper_node',
        output='screen',
        condition=IfCondition(gripper),
        parameters=[{
            'port': gripper_port,
            'allow_motion': ParameterValue(
                gripper_allow_motion, value_type=bool),
            'enable_motor_on_start': ParameterValue(
                gripper_enable_motor, value_type=bool),
            'can_fd': ParameterValue(gripper_can_fd, value_type=bool),
            'brs': ParameterValue(gripper_brs, value_type=bool),
            'joint_name': 'right_ctag_left_1_joint',
            'bitrate': 1_000_000,
            'data_bitrate': 5_000_000,
            'feedback_rate': 20.0,
            'position_tolerance': ParameterValue(
                gripper_position_tolerance, value_type=int),
            'command_max': ParameterValue(
                gripper_command_max, value_type=int),
            'motion_speed_percent': ParameterValue(
                gripper_speed_percent, value_type=int),
            'motion_torque_percent': ParameterValue(
                gripper_torque_percent, value_type=int),
            'motion_ramp_ms': ParameterValue(
                gripper_ramp_ms, value_type=int),
            'max_current': ParameterValue(
                gripper_max_current, value_type=int),
        }],
    )

    left_gripper_node = Node(
        package='dual_nero_control',
        executable='left_gripper_node',
        name='left_gripper_node',
        output='screen',
        condition=IfCondition(left_gripper),
        parameters=[{
            'port': left_gripper_port,
            'allow_motion': ParameterValue(
                left_gripper_allow_motion, value_type=bool),
            'enable_motor_on_start': ParameterValue(
                left_gripper_enable_motor, value_type=bool),
            'can_fd': ParameterValue(left_gripper_can_fd, value_type=bool),
            'brs': ParameterValue(left_gripper_brs, value_type=bool),
            'joint_name': 'left_ctag_left_1_joint',
            'bitrate': 1_000_000,
            'data_bitrate': 5_000_000,
            'feedback_rate': 20.0,
            'position_tolerance': ParameterValue(
                gripper_position_tolerance, value_type=int),
            'command_max': ParameterValue(
                gripper_command_max, value_type=int),
            'motion_speed_percent': ParameterValue(
                gripper_speed_percent, value_type=int),
            'motion_torque_percent': ParameterValue(
                gripper_torque_percent, value_type=int),
            'motion_ramp_ms': ParameterValue(
                gripper_ramp_ms, value_type=int),
            'max_current': ParameterValue(
                gripper_max_current, value_type=int),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dry_run', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'collision_padding', default_value='0.025',
            description='Extra collision clearance added around robot and scene '
                        'geometry in meters.'),
        DeclareLaunchArgument('command_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('trajectory_interpolation', default_value='linear'),
        DeclareLaunchArgument('settle_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('settle_tolerance_rad', default_value='0.01'),
        DeclareLaunchArgument('gripper', default_value='false'),
        DeclareLaunchArgument('gripper_allow_motion', default_value='false'),
        DeclareLaunchArgument('gripper_enable_motor', default_value='false'),
        DeclareLaunchArgument(
            'gripper_port',
            default_value=('/dev/serial/by-id/'
                           'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00')),
        DeclareLaunchArgument('gripper_can_fd', default_value='false'),
        DeclareLaunchArgument('gripper_brs', default_value='false'),
        DeclareLaunchArgument('gripper_speed_percent', default_value='25'),
        DeclareLaunchArgument('gripper_torque_percent', default_value='100'),
        DeclareLaunchArgument('gripper_ramp_ms', default_value='200'),
        DeclareLaunchArgument(
            'gripper_max_current', default_value='200',
            description='Vendor maximum-current register 0x0533 (1..1000)'),
        DeclareLaunchArgument(
            'gripper_position_tolerance', default_value='25'),
        DeclareLaunchArgument('gripper_command_max', default_value='990'),
        DeclareLaunchArgument('left_gripper', default_value='false'),
        DeclareLaunchArgument(
            'left_gripper_allow_motion', default_value='false'),
        DeclareLaunchArgument(
            'left_gripper_enable_motor', default_value='false'),
        DeclareLaunchArgument(
            'left_gripper_port',
            default_value=('/dev/serial/by-id/'
                           'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')),
        DeclareLaunchArgument('left_gripper_can_fd', default_value='false'),
        DeclareLaunchArgument('left_gripper_brs', default_value='false'),
        bridge,
        robot_state_publisher,
        gripper_node,
        left_gripper_node,
        move_group,
        rviz,
    ])
