from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


LEFT_PORT = ('/dev/serial/by-id/'
             'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00')
RIGHT_PORT = ('/dev/serial/by-id/'
              'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00')


def _gripper_node(side: str, port: LaunchConfiguration,
                  allow_motion: LaunchConfiguration,
                  enable_motor: LaunchConfiguration,
                  can_fd: LaunchConfiguration,
                  brs: LaunchConfiguration,
                  goal_timeout: LaunchConfiguration,
                  feedback_timeout: LaunchConfiguration,
                  latch_motion_timeout: LaunchConfiguration,
                  cancel_disables_motor: LaunchConfiguration,
                  motion_speed_percent: LaunchConfiguration,
                  motion_torque_percent: LaunchConfiguration,
                  motion_ramp_ms: LaunchConfiguration,
                  max_current: LaunchConfiguration,
                  position_tolerance: LaunchConfiguration,
                  command_max: LaunchConfiguration) -> Node:
    return Node(
        package='dual_nero_control',
        executable=f'{side}_gripper_node',
        name=f'{side}_gripper_node',
        output='screen',
        parameters=[{
            'port': port,
            'allow_motion': ParameterValue(allow_motion, value_type=bool),
            'enable_motor_on_start': ParameterValue(
                enable_motor, value_type=bool),
            'can_fd': ParameterValue(can_fd, value_type=bool),
            'brs': ParameterValue(brs, value_type=bool),
            'joint_name': f'{side}_ctag_left_1_joint',
            'bitrate': 1_000_000,
            'data_bitrate': 5_000_000,
            'feedback_rate': 20.0,
            'position_tolerance': ParameterValue(
                position_tolerance, value_type=int),
            'command_max': ParameterValue(command_max, value_type=int),
            'motion_speed_percent': ParameterValue(
                motion_speed_percent, value_type=int),
            'motion_torque_percent': ParameterValue(
                motion_torque_percent, value_type=int),
            'motion_ramp_ms': ParameterValue(
                motion_ramp_ms, value_type=int),
            'max_current': ParameterValue(max_current, value_type=int),
            'goal_timeout_sec': ParameterValue(
                goal_timeout, value_type=float),
            'feedback_timeout_sec': ParameterValue(
                feedback_timeout, value_type=float),
            'latch_motion_timeout': ParameterValue(
                latch_motion_timeout, value_type=bool),
            'cancel_disables_motor': ParameterValue(
                cancel_disables_motor, value_type=bool),
            'command_topic': f'/{side}_gripper/position_cmd',
            'enable_service': f'/{side}_gripper/enable',
            'action_name': f'/{side}_gripper_controller/gripper_cmd',
            'state_topic': f'/{side}_gripper/state',
            'position_topic': f'/{side}_gripper/position',
            'width_topic': f'/{side}_gripper/width',
        }],
    )


def generate_launch_description():
    left_port = LaunchConfiguration('left_port')
    right_port = LaunchConfiguration('right_port')
    allow_motion = LaunchConfiguration('allow_motion')
    enable_motor = LaunchConfiguration('enable_motor')
    can_fd = LaunchConfiguration('can_fd')
    brs = LaunchConfiguration('brs')
    goal_timeout = LaunchConfiguration('goal_timeout_sec')
    feedback_timeout = LaunchConfiguration('feedback_timeout_sec')
    latch_motion_timeout = LaunchConfiguration('latch_motion_timeout')
    cancel_disables_motor = LaunchConfiguration('cancel_disables_motor')
    motion_speed_percent = LaunchConfiguration('motion_speed_percent')
    motion_torque_percent = LaunchConfiguration('motion_torque_percent')
    motion_ramp_ms = LaunchConfiguration('motion_ramp_ms')
    max_current = LaunchConfiguration('max_current')
    position_tolerance = LaunchConfiguration('position_tolerance')
    command_max = LaunchConfiguration('command_max')

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_port', default_value=LEFT_PORT,
            description='ZQWL serial device for the left gripper'),
        DeclareLaunchArgument(
            'right_port', default_value=RIGHT_PORT,
            description='ZQWL serial device for the right gripper'),
        DeclareLaunchArgument(
            'allow_motion', default_value='false',
            description='Allow enable/position writes to both grippers'),
        DeclareLaunchArgument(
            'enable_motor', default_value='false',
            description='Request motor enable when the nodes start'),
        DeclareLaunchArgument(
            'can_fd', default_value='false',
            description='Use CAN-FD frames instead of classic CAN'),
        DeclareLaunchArgument(
            'brs', default_value='false',
            description='Set the BRS bit on CAN-FD frames'),
        DeclareLaunchArgument(
            'goal_timeout_sec', default_value='10.0',
            description='Maximum time for one gripper motion command'),
        DeclareLaunchArgument(
            'feedback_timeout_sec', default_value='1.0',
            description='Maximum feedback age while enabled or moving'),
        DeclareLaunchArgument(
            'latch_motion_timeout', default_value='true',
            description='Fault and disable the motor when motion times out'),
        DeclareLaunchArgument(
            'cancel_disables_motor', default_value='true',
            description='Fault and disable the motor on action cancellation'),
        DeclareLaunchArgument(
            'motion_speed_percent', default_value='25',
            description='Vendor speed register 0x0104, percent of max speed'),
        DeclareLaunchArgument(
            'motion_torque_percent', default_value='100',
            description='Vendor torque-limit register 0x0105, percent; '
                        'this is not closed-loop force control'),
        DeclareLaunchArgument(
            'motion_ramp_ms', default_value='200',
            description='Vendor acceleration-ramp register 0x0106, msec'),
        DeclareLaunchArgument(
            'max_current', default_value='200',
            description='Vendor maximum-current register 0x0533, 1..1000; '
                        'raise in verified steps if motion still stalls'),
        DeclareLaunchArgument(
            'position_tolerance', default_value='25',
            description='Raw position counts treated as reaching the target'),
        DeclareLaunchArgument(
            'command_max', default_value='990',
            description='Maximum closing position command for both grippers; '
                        'feedback still reports the raw 0..1000 range'),
        _gripper_node(
            'left', left_port, allow_motion, enable_motor, can_fd, brs,
            goal_timeout, feedback_timeout, latch_motion_timeout,
            cancel_disables_motor, motion_speed_percent,
            motion_torque_percent, motion_ramp_ms, max_current,
            position_tolerance, command_max),
        _gripper_node(
            'right', right_port, allow_motion, enable_motor, can_fd, brs,
            goal_timeout, feedback_timeout, latch_motion_timeout,
            cancel_disables_motor, motion_speed_percent,
            motion_torque_percent, motion_ramp_ms, max_current,
            position_tolerance, command_max),
    ])
