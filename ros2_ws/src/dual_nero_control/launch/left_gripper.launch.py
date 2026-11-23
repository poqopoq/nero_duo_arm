from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PORT = ('/dev/serial/by-id/'
        'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')


def generate_launch_description():
    transport = LaunchConfiguration('transport')
    port = LaunchConfiguration('port')
    can_interface = LaunchConfiguration('can_interface')
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
            'transport', default_value='zqwl',
            description='CAN adapter backend: zqwl or socketcan'),
        DeclareLaunchArgument(
            'port', default_value=PORT,
            description='ZQWL CAN-FD adapter serial device'),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='SocketCAN interface used when transport=socketcan'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='Allow OmniPicker 3 motion commands'),
        DeclareLaunchArgument(
            'enable_motor', default_value='false',
            description='Mark the OP3 command stream enabled at startup'),
        DeclareLaunchArgument(
            'can_fd', default_value='true',
            description='Use CAN-FD frames (OmniPicker 3 CAN data phase)'),
        DeclareLaunchArgument(
            'brs', default_value='true',
            description='Set BRS for the 5 Mbps CAN-FD data phase'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='10.0',
            description='Maximum time for one gripper motion command'),
        DeclareLaunchArgument(
            'feedback_timeout_sec', default_value='1.0',
            description='Maximum upstream status age during motion'),
        DeclareLaunchArgument(
            'latch_motion_timeout', default_value='true',
            description='Latch a fault when feedback is lost during motion'),
        DeclareLaunchArgument(
            'cancel_stops_motion', default_value='true',
            description='Stop the OP3 command stream when a goal is canceled'),
        DeclareLaunchArgument(
            'motion_speed_percent', default_value='50',
            description='OP3 velocity byte as a percentage of maximum'),
        DeclareLaunchArgument(
            'motion_force_percent', default_value='100',
            description='OP3 force/torque byte as a percentage of maximum'),
        DeclareLaunchArgument(
            'motion_acceleration_percent', default_value='100',
            description='OP3 acceleration byte as a percentage of maximum'),
        DeclareLaunchArgument(
            'motion_deceleration_percent', default_value='100',
            description='OP3 deceleration byte as a percentage of maximum'),
        DeclareLaunchArgument(
            'position_tolerance', default_value='8',
            description='ROS position counts (0..1000) treated as at target'),
        DeclareLaunchArgument(
            'command_max', default_value='1000',
            description='Maximum closing command; 0 is open and 1000 is closed'),
        Node(
            package='dual_nero_control',
            executable='left_gripper_node',
            name='left_gripper_node',
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
                'joint_name': 'left_op3_wide1_joint',
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
            }],
        ),
    ])
