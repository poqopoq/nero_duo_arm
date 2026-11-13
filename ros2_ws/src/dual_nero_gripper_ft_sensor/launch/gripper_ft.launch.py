from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    serial_port = LaunchConfiguration("serial_port")
    node_name = LaunchConfiguration("node_name")
    data_topic = LaunchConfiguration("data_topic")
    raw_topic = LaunchConfiguration("raw_topic")
    baud_rate = LaunchConfiguration("baud_rate")
    frame_id = LaunchConfiguration("frame_id")
    sensor_id = LaunchConfiguration("sensor_id")
    expected_serial_number = LaunchConfiguration("expected_serial_number")
    query_serial_number_on_start = LaunchConfiguration(
        "query_serial_number_on_start")
    require_serial_match = LaunchConfiguration("require_serial_match")
    publish_raw = LaunchConfiguration("publish_raw")
    qos_reliability = LaunchConfiguration("qos_reliability")
    qos_depth = LaunchConfiguration("qos_depth")
    response_timeout = LaunchConfiguration("response_timeout")
    reconnect_interval = LaunchConfiguration("reconnect_interval")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port", default_value="/dev/ttyUSB0",
            description="Sensor UART device"),
        DeclareLaunchArgument(
            "node_name", default_value="gripper_ft_node",
            description="ROS node name and service namespace"),
        DeclareLaunchArgument(
            "data_topic", default_value="/gripper_ft/data",
            description="SixAxisForce output topic"),
        DeclareLaunchArgument(
            "raw_topic", default_value="/gripper_ft/raw",
            description="Raw measurement payload topic"),
        DeclareLaunchArgument(
            "baud_rate", default_value="921600",
            description="Sensor UART baud rate"),
        DeclareLaunchArgument(
            "frame_id", default_value="gripper_ft_sensor",
            description="frame_id used in SixAxisForce headers"),
        DeclareLaunchArgument(
            "sensor_id", default_value="gripper_ft_node",
            description="Logical sensor name stored in every message"),
        DeclareLaunchArgument(
            "expected_serial_number", default_value="",
            description="Expected hardware SN; empty accepts any sensor"),
        DeclareLaunchArgument(
            "query_serial_number_on_start", default_value="true",
            description="Read the hardware SN with command 0x04 on connect"),
        DeclareLaunchArgument(
            "require_serial_match", default_value="false",
            description="Refuse to publish unless the expected SN matches"),
        DeclareLaunchArgument(
            "publish_raw", default_value="false",
            description="Also publish raw measurement payload bytes"),
        DeclareLaunchArgument(
            "qos_reliability", default_value="best_effort",
            description="Force data QoS: best_effort (low latency) or reliable"),
        DeclareLaunchArgument(
            "qos_depth", default_value="1",
            description="Force data QoS history depth"),
        DeclareLaunchArgument(
            "response_timeout", default_value="1.0",
            description="Command response timeout in seconds"),
        DeclareLaunchArgument(
            "reconnect_interval", default_value="2.0",
            description="Serial reconnect interval in seconds"),
        Node(
            package="dual_nero_gripper_ft_sensor",
            executable="gripper_ft_node",
            name=node_name,
            output="screen",
            parameters=[{
                "serial_port": ParameterValue(
                    serial_port, value_type=str),
                "data_topic": ParameterValue(data_topic, value_type=str),
                "raw_topic": ParameterValue(raw_topic, value_type=str),
                "baud_rate": ParameterValue(baud_rate, value_type=int),
                "frame_id": ParameterValue(frame_id, value_type=str),
                "sensor_id": ParameterValue(sensor_id, value_type=str),
                "expected_serial_number": ParameterValue(
                    expected_serial_number, value_type=str),
                "query_serial_number_on_start": ParameterValue(
                    query_serial_number_on_start, value_type=bool),
                "require_serial_match": ParameterValue(
                    require_serial_match, value_type=bool),
                "publish_raw": ParameterValue(
                    publish_raw, value_type=bool),
                "qos_reliability": ParameterValue(
                    qos_reliability, value_type=str),
                "qos_depth": ParameterValue(
                    qos_depth, value_type=int),
                "response_timeout": ParameterValue(
                    response_timeout, value_type=float),
                "reconnect_interval": ParameterValue(
                    reconnect_interval, value_type=float),
            }],
        ),
    ])
