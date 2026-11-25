from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    device_id = LaunchConfiguration('device_id')
    combined_query = LaunchConfiguration('combined_query')
    rate_hz = LaunchConfiguration('rate_hz')
    node_name = LaunchConfiguration('node_name')
    data_topic_face_a = LaunchConfiguration('data_topic_face_a')
    data_topic_face_b = LaunchConfiguration('data_topic_face_b')
    frame_id_face_a = LaunchConfiguration('frame_id_face_a')
    frame_id_face_b = LaunchConfiguration('frame_id_face_b')
    sensor_id_face_a = LaunchConfiguration('sensor_id_face_a')
    sensor_id_face_b = LaunchConfiguration('sensor_id_face_b')
    qos_reliability = LaunchConfiguration('qos_reliability')
    qos_depth = LaunchConfiguration('qos_depth')
    response_timeout = LaunchConfiguration('response_timeout')
    reconnect_interval = LaunchConfiguration('reconnect_interval')
    expected_sensor_count = LaunchConfiguration('expected_sensor_count')
    rate_log_period = LaunchConfiguration('rate_log_period')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/'
                'usb-FTDI_FT232R_USB_UART_BH00URZP-if00-port0'),
            description='OP3 RS485 adapter device'),
        DeclareLaunchArgument(
            'baud_rate', default_value='460800',
            description='OP3 RS485 baud rate (vendor default 460800)'),
        DeclareLaunchArgument(
            'device_id', default_value='1',
            description='OmniPicker device id'),
        DeclareLaunchArgument(
            'combined_query', default_value='true',
            description='Read both faces in one SUB_ALL transaction'),
        DeclareLaunchArgument(
            'rate_hz', default_value='0.0',
            description='Readout throttle in Hz (0 = as fast as possible)'),
        DeclareLaunchArgument(
            'node_name', default_value='gripper_ft_rs485_node',
            description='ROS node name'),
        DeclareLaunchArgument(
            'data_topic_face_a', default_value='/gripper_ft/a',
            description='SixAxisForce topic for face A'),
        DeclareLaunchArgument(
            'data_topic_face_b', default_value='/gripper_ft/b',
            description='SixAxisForce topic for face B'),
        DeclareLaunchArgument(
            'frame_id_face_a', default_value='gripper_ft_face_a',
            description='frame_id for face A'),
        DeclareLaunchArgument(
            'frame_id_face_b', default_value='gripper_ft_face_b',
            description='frame_id for face B'),
        DeclareLaunchArgument(
            'sensor_id_face_a', default_value='face_a',
            description='sensor_id stored in face A messages'),
        DeclareLaunchArgument(
            'sensor_id_face_b', default_value='face_b',
            description='sensor_id stored in face B messages'),
        DeclareLaunchArgument(
            'qos_reliability', default_value='best_effort',
            description='Force data QoS: best_effort or reliable'),
        DeclareLaunchArgument(
            'qos_depth', default_value='1',
            description='Force data QoS history depth'),
        DeclareLaunchArgument(
            'response_timeout', default_value='0.05',
            description='Per-transaction response timeout in seconds'),
        DeclareLaunchArgument(
            'reconnect_interval', default_value='1.0',
            description='Reconnect delay after a serial error'),
        DeclareLaunchArgument(
            'expected_sensor_count', default_value='2',
            description='Expected tactile sensor count (0 disables check)'),
        DeclareLaunchArgument(
            'rate_log_period', default_value='5.0',
            description='Seconds between rate log lines'),
        Node(
            package='dual_nero_gripper_ft_sensor',
            executable='gripper_ft_rs485_node',
            name=node_name,
            output='screen',
            parameters=[{
                'serial_port': ParameterValue(serial_port, value_type=str),
                'baud_rate': ParameterValue(baud_rate, value_type=int),
                'device_id': ParameterValue(device_id, value_type=int),
                'combined_query': ParameterValue(
                    combined_query, value_type=bool),
                'rate_hz': ParameterValue(rate_hz, value_type=float),
                'data_topic_face_a': ParameterValue(
                    data_topic_face_a, value_type=str),
                'data_topic_face_b': ParameterValue(
                    data_topic_face_b, value_type=str),
                'frame_id_face_a': ParameterValue(
                    frame_id_face_a, value_type=str),
                'frame_id_face_b': ParameterValue(
                    frame_id_face_b, value_type=str),
                'sensor_id_face_a': ParameterValue(
                    sensor_id_face_a, value_type=str),
                'sensor_id_face_b': ParameterValue(
                    sensor_id_face_b, value_type=str),
                'qos_reliability': ParameterValue(
                    qos_reliability, value_type=str),
                'qos_depth': ParameterValue(qos_depth, value_type=int),
                'response_timeout': ParameterValue(
                    response_timeout, value_type=float),
                'reconnect_interval': ParameterValue(
                    reconnect_interval, value_type=float),
                'expected_sensor_count': ParameterValue(
                    expected_sensor_count, value_type=int),
                'rate_log_period': ParameterValue(
                    rate_log_period, value_type=float),
            }],
        ),
    ])
