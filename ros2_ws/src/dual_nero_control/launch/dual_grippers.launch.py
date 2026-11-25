from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LEFT_PORT = ('/dev/serial/by-id/'
             'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')
RIGHT_PORT = ('/dev/serial/by-id/'
              'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00')


def _gripper_node(
        side, transport, port, can_interface,
        allow_motion, enable_motor, can_fd, brs,
        goal_timeout, feedback_timeout, latch_motion_timeout,
        cancel_stops_motion, motion_speed_percent,
        motion_force_percent, motion_acceleration_percent,
        motion_deceleration_percent, position_tolerance,
        command_max) -> Node:
    return Node(
        package='dual_nero_control',
        executable=f'{side}_gripper_node',
        name=f'{side}_gripper_node',
        output='screen',
        parameters=[{
            'transport': transport,
            'port': port,
            'can_interface': can_interface,
            'allow_motion': ParameterValue(allow_motion, value_type=bool),
            'enable_motor_on_start': ParameterValue(
                enable_motor, value_type=bool),
            'can_fd': ParameterValue(can_fd, value_type=bool),
            'brs': ParameterValue(brs, value_type=bool),
            'joint_name': f'{side}_op3_joint1',
            'bitrate': 1_000_000,
            'data_bitrate': 5_000_000,
            'parallel_gripper_width_mm': 120.0,
            'feedback_rate': 20.0,
            'position_tolerance': ParameterValue(
                position_tolerance, value_type=int),
            'command_max': ParameterValue(command_max, value_type=int),
            'motion_speed_percent': ParameterValue(
                motion_speed_percent, value_type=int),
            'motion_force_percent': ParameterValue(
                motion_force_percent, value_type=int),
            'motion_acceleration_percent': ParameterValue(
                motion_acceleration_percent, value_type=int),
            'motion_deceleration_percent': ParameterValue(
                motion_deceleration_percent, value_type=int),
            'goal_timeout_sec': ParameterValue(
                goal_timeout, value_type=float),
            'feedback_timeout_sec': ParameterValue(
                feedback_timeout, value_type=float),
            'latch_motion_timeout': ParameterValue(
                latch_motion_timeout, value_type=bool),
            'cancel_stops_motion': ParameterValue(
                cancel_stops_motion, value_type=bool),
            'command_topic': f'/{side}_gripper/position_cmd',
            'enable_service': f'/{side}_gripper/enable',
            'action_name': f'/{side}_gripper_controller/gripper_cmd',
            'state_topic': f'/{side}_gripper/state',
            'position_topic': f'/{side}_gripper/position',
            'width_topic': f'/{side}_gripper/width',
        }],
    )


def generate_launch_description():
    left_transport = LaunchConfiguration('left_transport')
    right_transport = LaunchConfiguration('right_transport')
    left_port = LaunchConfiguration('left_port')
    right_port = LaunchConfiguration('right_port')
    left_can_interface = LaunchConfiguration('left_can_interface')
    right_can_interface = LaunchConfiguration('right_can_interface')
    allow_motion = LaunchConfiguration('allow_motion')
    enable_motor = LaunchConfiguration('enable_motor')
    can_fd = LaunchConfiguration('can_fd')
    brs = LaunchConfiguration('brs')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    feedback_timeout = LaunchConfiguration('feedback_timeout_sec')
    latch_motion_timeout = LaunchConfiguration('latch_motion_timeout')
    cancel_stops_motion = LaunchConfiguration('cancel_stops_motion')
    motion_speed_percent = LaunchConfiguration('motion_speed_percent')
    motion_force_percent = LaunchConfiguration('motion_force_percent')
    motion_acceleration_percent = LaunchConfiguration(
        'motion_acceleration_percent')
    motion_deceleration_percent = LaunchConfiguration(
        'motion_deceleration_percent')
    position_tolerance = LaunchConfiguration('position_tolerance')
    command_max = LaunchConfiguration('command_max')

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_transport', default_value='zqwl',
            description='Left backend: zqwl or socketcan'),
        DeclareLaunchArgument(
            'right_transport', default_value='zqwl',
            description='Right backend: zqwl or socketcan'),
        DeclareLaunchArgument(
            'left_port', default_value=LEFT_PORT,
            description='ZQWL CAN-FD adapter serial device for the left gripper'),
        DeclareLaunchArgument(
            'right_port', default_value=RIGHT_PORT,
            description='ZQWL CAN-FD adapter serial device for the right gripper'),
        DeclareLaunchArgument(
            'left_can_interface', default_value='can0',
            description='SocketCAN interface for the left gripper'),
        DeclareLaunchArgument(
            'right_can_interface', default_value='can1',
            description='SocketCAN interface for the right gripper'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='Allow motion commands to both OmniPicker 3 grippers'),
        DeclareLaunchArgument(
            'enable_motor', default_value='false',
            description='Mark both OP3 command streams enabled at startup'),
        DeclareLaunchArgument(
            'can_fd', default_value='true',
            description='Use CAN-FD frames'),
        DeclareLaunchArgument(
            'brs', default_value='true',
            description='Set BRS for the 5 Mbps data phase'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='10.0'),
        DeclareLaunchArgument(
            'feedback_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument(
            'latch_motion_timeout', default_value='true'),
        DeclareLaunchArgument(
            'cancel_stops_motion', default_value='true'),
        DeclareLaunchArgument(
            'motion_speed_percent', default_value='50'),
        DeclareLaunchArgument(
            'motion_force_percent', default_value='100'),
        DeclareLaunchArgument(
            'motion_acceleration_percent', default_value='100'),
        DeclareLaunchArgument(
            'motion_deceleration_percent', default_value='100'),
        DeclareLaunchArgument(
            'position_tolerance', default_value='8'),
        DeclareLaunchArgument(
            'command_max', default_value='1000'),
        _gripper_node(
            'left', left_transport, left_port, left_can_interface,
            allow_motion, enable_motor, can_fd, brs, goal_timeout,
            feedback_timeout, latch_motion_timeout, cancel_stops_motion,
            motion_speed_percent, motion_force_percent,
            motion_acceleration_percent, motion_deceleration_percent,
            position_tolerance, command_max),
        _gripper_node(
            'right', right_transport, right_port, right_can_interface,
            allow_motion, enable_motor, can_fd, brs, goal_timeout,
            feedback_timeout, latch_motion_timeout, cancel_stops_motion,
            motion_speed_percent, motion_force_percent,
            motion_acceleration_percent, motion_deceleration_percent,
            position_tolerance, command_max),
    ])
