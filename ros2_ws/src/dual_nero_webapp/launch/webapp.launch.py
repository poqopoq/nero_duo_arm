import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_recover_script() -> Path:
    """Find the recovery wrapper in both the source and installed layouts."""
    override = os.environ.get('NERO_RECOVER_JOINT_LIMITS_SCRIPT')
    if override:
        path = Path(override).expanduser()
        if not path.is_file():
            raise RuntimeError(
                f'NERO_RECOVER_JOINT_LIMITS_SCRIPT does not exist: {path}')
        return path
    launch_file = Path(__file__).resolve()
    for ancestor in launch_file.parents:
        candidate = (
            ancestor / 'deployment' / 'scripts' / 'recover_joint_limits.sh')
        if candidate.is_file():
            return candidate
    raise RuntimeError(
        'recover_joint_limits.sh not found under any parent of '
        f'{launch_file}; set NERO_RECOVER_JOINT_LIMITS_SCRIPT to override')


RECOVER_SCRIPT = _resolve_recover_script()


def generate_launch_description():
    control_share = get_package_share_directory('dual_nero_control')
    cameras_share = get_package_share_directory('dual_nero_cameras')
    description_share = get_package_share_directory('dual_nero_description')
    moveit_share = get_package_share_directory('dual_nero_moveit_config')
    webapp_share = get_package_share_directory('dual_nero_webapp')
    xacro_path = os.path.join(
        description_share, 'urdf', 'two_nero_tabletop.urdf.xacro')
    support_pose_path = os.path.join(
        webapp_share, 'config', 'support_pose.yaml')

    start_drivers = LaunchConfiguration('start_drivers')
    start_grippers = LaunchConfiguration('start_grippers')
    start_cameras = LaunchConfiguration('start_cameras')
    start_moveit = LaunchConfiguration('start_moveit')
    moveit_dry_run = LaunchConfiguration('moveit_dry_run')
    moveit_rviz = LaunchConfiguration('moveit_rviz')
    moveit_velocity_scaling = LaunchConfiguration('moveit_velocity_scaling')
    moveit_acceleration_scaling = LaunchConfiguration(
        'moveit_acceleration_scaling')
    collision_padding = LaunchConfiguration('collision_padding')
    trajectory_command_rate_hz = LaunchConfiguration(
        'trajectory_command_rate_hz')
    trajectory_interpolation = LaunchConfiguration('trajectory_interpolation')
    trajectory_settle_timeout_sec = LaunchConfiguration(
        'trajectory_settle_timeout_sec')
    trajectory_settle_tolerance_rad = LaunchConfiguration(
        'trajectory_settle_tolerance_rad')
    enable_cameras = LaunchConfiguration('enable_cameras')

    webapp = Node(
        package='dual_nero_webapp',
        executable='webapp_node',
        name='dual_nero_webapp',
        output='screen',
        parameters=[{
            'left_ns': LaunchConfiguration('left_ns'),
            'right_ns': LaunchConfiguration('right_ns'),
            'http_host': LaunchConfiguration('http_host'),
            'http_port': LaunchConfiguration('http_port'),
            'initial_gate_open': LaunchConfiguration('initial_gate_open'),
            'enable_cameras': enable_cameras,
            'depth_min_m': LaunchConfiguration('depth_min_m'),
            'depth_max_m': LaunchConfiguration('depth_max_m'),
            'use_compressed_color': LaunchConfiguration('use_compressed_color'),
            'scene_color_topic': LaunchConfiguration('scene_color_topic'),
            'scene_color_compressed_topic': LaunchConfiguration(
                'scene_color_compressed_topic'),
            'scene_depth_topic': LaunchConfiguration('scene_depth_topic'),
            'left_wrist_color_topic': LaunchConfiguration('left_wrist_color_topic'),
            'left_wrist_color_compressed_topic': LaunchConfiguration(
                'left_wrist_color_compressed_topic'),
            'left_wrist_depth_topic': LaunchConfiguration('left_wrist_depth_topic'),
            'right_wrist_color_topic': LaunchConfiguration('right_wrist_color_topic'),
            'right_wrist_color_compressed_topic': LaunchConfiguration(
                'right_wrist_color_compressed_topic'),
            'right_wrist_depth_topic': LaunchConfiguration('right_wrist_depth_topic'),
            'support_pose_file': support_pose_path,
            'joint16_home_file': LaunchConfiguration('joint16_home_file'),
            'urdf_xacro': xacro_path,
            'trajectory_cancel_service': LaunchConfiguration(
                'trajectory_cancel_service'),
            'trajectory_clear_cancel_service': LaunchConfiguration(
                'trajectory_clear_cancel_service'),
            'moveit_dry_run': moveit_dry_run,
            'inference_rollout_script': LaunchConfiguration(
                'inference_rollout_script'),
            'inference_bridge_socket': LaunchConfiguration(
                'inference_bridge_socket'),
            'inference_default_task': LaunchConfiguration(
                'inference_default_task'),
            'inference_policy_port': LaunchConfiguration(
                'inference_policy_port'),
            'recover_on_enable': LaunchConfiguration('recover_joint_limits'),
            'joint_limit_recovery_script': str(RECOVER_SCRIPT),
            'joint_limit_recovery_timeout_sec': 180.0,
            'moveit_velocity_scaling': moveit_velocity_scaling,
            'moveit_acceleration_scaling': moveit_acceleration_scaling,
            'table_width': LaunchConfiguration('table_width'),
            'table_depth': LaunchConfiguration('table_depth'),
            'table_thickness': LaunchConfiguration('table_thickness'),
            'base_spacing': LaunchConfiguration('base_spacing'),
            'front_edge_offset': LaunchConfiguration('front_edge_offset'),
            'left_base_yaw': LaunchConfiguration('left_base_yaw'),
            'right_base_yaw': LaunchConfiguration('right_base_yaw'),
            'include_gripper': LaunchConfiguration('include_gripper'),
            'allow_gripper_motion': LaunchConfiguration(
                'gripper_allow_motion'),
            'left_gripper_prefix': '/left_gripper',
            'right_gripper_prefix': '/right_gripper',
            'gripper_position_min': 0,
            'gripper_position_max': 1000,
            'gripper_feedback_timeout_sec': 1.5,
        }],
    )

    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'dual_nero_drivers.launch.py')),
        condition=IfCondition(start_drivers),
        launch_arguments={
            'can_left': LaunchConfiguration('can_left'),
            'can_right': LaunchConfiguration('can_right'),
            'speed_percent': LaunchConfiguration('speed_percent'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'recover_joint_limits': LaunchConfiguration(
                'recover_joint_limits'),
            'recover_joint_limits_delay_sec': LaunchConfiguration(
                'recover_joint_limits_delay_sec'),
        }.items(),
    )

    grippers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(control_share, 'launch', 'dual_grippers.launch.py')),
        condition=IfCondition(start_grippers),
        launch_arguments={
            'left_transport': LaunchConfiguration('gripper_transport'),
            'right_transport': LaunchConfiguration('gripper_transport'),
            'left_port': LaunchConfiguration('left_gripper_port'),
            'right_port': LaunchConfiguration('right_gripper_port'),
            'left_can_interface': LaunchConfiguration(
                'left_gripper_can_interface'),
            'right_can_interface': LaunchConfiguration(
                'right_gripper_can_interface'),
            'allow_motion': LaunchConfiguration('gripper_allow_motion'),
            'enable_motor': LaunchConfiguration('gripper_enable_motor'),
            'can_fd': LaunchConfiguration('gripper_can_fd'),
            'brs': LaunchConfiguration('gripper_brs'),
            'goal_timeout_sec': LaunchConfiguration(
                'gripper_goal_timeout_sec'),
            'feedback_timeout_sec': LaunchConfiguration(
                'gripper_driver_feedback_timeout_sec'),
            'latch_motion_timeout': LaunchConfiguration(
                'gripper_latch_motion_timeout'),
            'cancel_stops_motion': LaunchConfiguration(
                'gripper_cancel_disables_motor'),
            'motion_speed_percent': LaunchConfiguration(
                'gripper_speed_percent'),
            'motion_force_percent': LaunchConfiguration(
                'gripper_torque_percent'),
            'position_tolerance': LaunchConfiguration(
                'gripper_position_tolerance'),
            'command_max': LaunchConfiguration('gripper_command_max'),
        }.items(),
    )

    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cameras_share, 'launch', 'all_cameras.launch.py')),
        condition=IfCondition(start_cameras),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                moveit_share, 'launch', 'moveit_execution.launch.py')),
        condition=IfCondition(start_moveit),
        launch_arguments={
            'dry_run': moveit_dry_run,
            'rviz': moveit_rviz,
            'collision_padding': collision_padding,
            'command_rate_hz': trajectory_command_rate_hz,
            'trajectory_interpolation': trajectory_interpolation,
            'settle_timeout_sec': trajectory_settle_timeout_sec,
            'settle_tolerance_rad': trajectory_settle_tolerance_rad,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_ns', default_value='left',
            description='ROS namespace for the left arm driver.'),
        DeclareLaunchArgument(
            'right_ns', default_value='right',
            description='ROS namespace for the right arm driver.'),
        DeclareLaunchArgument(
            'http_host', default_value='0.0.0.0',
            description='HTTP bind address. Use 127.0.0.1 to keep the UI local.'),
        DeclareLaunchArgument(
            'http_port', default_value='8080', description='HTTP port.'),
        DeclareLaunchArgument(
            'initial_gate_open', default_value='false',
            description='Only controls the webapp-side interlock; the driver '
                        'control gate still starts closed.'),
        DeclareLaunchArgument(
            'start_drivers', default_value='false',
            description='Also launch the two namespaced NERO drivers.'),
        DeclareLaunchArgument(
            'start_grippers', default_value='false',
            description='Also launch both OmniPicker 3 CAN-FD gripper drivers. '
                        'Defaults off because the grippers may be disconnected; '
                        'set true only when both CAN grippers are present.'),
        DeclareLaunchArgument(
            'gripper_allow_motion', default_value='false',
            description='Allow gripper enable/position writes in both the '
                        'webapp and the gripper drivers. Keep false until the '
                        'jaw workspace is clear.'),
        DeclareLaunchArgument(
            'gripper_enable_motor', default_value='false',
            description='Request gripper motor enable at driver startup. The '
                        'web UI can still auto-enable when a move is requested.'),
        DeclareLaunchArgument(
            'gripper_transport', default_value='zqwl',
            description='CAN backend for both grippers: zqwl or socketcan.'),
        DeclareLaunchArgument(
            'gripper_can_fd', default_value='true',
            description='Use CAN-FD frames for both OmniPicker 3 grippers.'),
        DeclareLaunchArgument(
            'gripper_brs', default_value='true',
            description='Set BRS for the 5 Mbps OmniPicker 3 data phase.'),
        DeclareLaunchArgument(
            'gripper_goal_timeout_sec', default_value='10.0',
            description='Driver timeout for one gripper motion command.'),
        DeclareLaunchArgument(
            'gripper_driver_feedback_timeout_sec', default_value='1.0',
            description='Maximum gripper feedback age while enabled or moving.'),
        DeclareLaunchArgument(
            'gripper_latch_motion_timeout', default_value='true',
            description='Latch a fault when OP3 status feedback times out.'),
        DeclareLaunchArgument(
            'gripper_cancel_disables_motor', default_value='true',
            description='Stop the OP3 command stream when an action is canceled.'),
        DeclareLaunchArgument(
            'gripper_speed_percent', default_value='50',
            description='OmniPicker 3 velocity byte (1..100 percent).'),
        DeclareLaunchArgument(
            'gripper_torque_percent', default_value='100',
            description='OmniPicker 3 force/torque byte (1..100 percent). '
                        'This is not a calibrated N/Nm force reading.'),
        DeclareLaunchArgument(
            'gripper_ramp_ms', default_value='200',
            description='Deprecated compatibility argument; ignored by OP3.'),
        DeclareLaunchArgument(
            'gripper_max_current', default_value='200',
            description='Deprecated compatibility argument; ignored by OP3.'),
        DeclareLaunchArgument(
            'gripper_position_tolerance', default_value='8',
            description='Raw position counts treated as reaching the target.'),
        DeclareLaunchArgument(
            'gripper_command_max', default_value='1000',
            description='Maximum closing command; 0 is open and 1000 is closed.'),
        DeclareLaunchArgument(
            'left_gripper_can_interface', default_value='can0',
            description='SocketCAN interface for the left OP3.'),
        DeclareLaunchArgument(
            'right_gripper_can_interface', default_value='can1',
            description='SocketCAN interface for the right OP3.'),
        DeclareLaunchArgument(
            'left_gripper_port',
            default_value=('/dev/serial/by-id/'
                           'usb-ZQWL-CANFD_ZQWL-CANFD_6A646D9E210A-if00'),
            description='ZQWL serial device for the left gripper.'),
        DeclareLaunchArgument(
            'right_gripper_port',
            default_value=('/dev/serial/by-id/'
                           'usb-ZQWL-CANFD_ZQWL-CANFD_6A649697220A-if00'),
            description='ZQWL serial device for the right gripper.'),
        DeclareLaunchArgument(
            'start_cameras', default_value='false',
            description='Also launch the three RealSense camera nodes.'),
        DeclareLaunchArgument(
            'start_moveit', default_value='false',
            description='Also launch MoveIt move_group and the trajectory bridge. '
                        'The webapp uses /move_action for all motions.'),
        DeclareLaunchArgument(
            'moveit_dry_run', default_value='true',
            description='When starting MoveIt, simulate execution without '
                        'commanding the drivers. Set false only for validated '
                        'real-hardware motion.'),
        DeclareLaunchArgument(
            'inference_rollout_script', default_value='',
            description='Interactive joint16 rollout wrapper used by the '
                        'Inference panel. Empty auto-discovers the deployment script.'),
        DeclareLaunchArgument(
            'inference_bridge_socket',
            default_value='/tmp/nero_bimanual_vla_bridge.sock',
            description='Unix socket used to probe and clear the VLA bridge.'),
        DeclareLaunchArgument(
            'inference_default_task',
            default_value='Use the right arm to put apple into white box.',
            description='Initial task sent to the interactive rollout.'),
        DeclareLaunchArgument(
            'inference_policy_port', default_value='7447',
            description='Local policy server port checked before launch.'),
        DeclareLaunchArgument(
            'joint16_home_file', default_value='',
            description='Native joint16 training/reset pose YAML. Empty auto-discovers '
                        'deployment/config/joint16_home.yaml.'),
        DeclareLaunchArgument(
            'moveit_rviz', default_value='false',
            description='Open RViz when start_moveit is true.'),
        DeclareLaunchArgument(
            'moveit_velocity_scaling', default_value='0.05',
            description='MoveIt maximum joint velocity scaling factor.'),
        DeclareLaunchArgument(
            'moveit_acceleration_scaling', default_value='0.05',
            description='MoveIt maximum joint acceleration scaling factor.'),
        DeclareLaunchArgument(
            'collision_padding', default_value='0.025',
            description='Extra MoveIt collision clearance in meters.'),
        DeclareLaunchArgument(
            'trajectory_command_rate_hz', default_value='50.0',
            description='Trajectory bridge command publishing rate in Hz.'),
        DeclareLaunchArgument(
            'trajectory_interpolation', default_value='linear',
            description='Trajectory bridge interpolation: linear or cubic.'),
        DeclareLaunchArgument(
            'trajectory_settle_timeout_sec', default_value='1.0',
            description='Extra time to hold the final target until settled.'),
        DeclareLaunchArgument(
            'trajectory_settle_tolerance_rad', default_value='0.01',
            description='Final target tracking tolerance in radians.'),
        DeclareLaunchArgument(
            'trajectory_cancel_service',
            default_value='/dual_nero_trajectory_bridge/cancel_motion',
            description='Service used to stop the active trajectory bridge goal.'),
        DeclareLaunchArgument(
            'trajectory_clear_cancel_service',
            default_value='/dual_nero_trajectory_bridge/clear_cancel',
            description='Service used to clear the bridge cancellation latch.'),
        DeclareLaunchArgument(
            'enable_cameras', default_value='true',
            description='Subscribe to the configured ROS camera topics.'),
        DeclareLaunchArgument('depth_min_m', default_value='0.05'),
        DeclareLaunchArgument('depth_max_m', default_value='2.0'),
        DeclareLaunchArgument(
            'use_compressed_color', default_value='true',
            description='Subscribe to relayed JPEG color topics for stable 30 FPS.'),
        DeclareLaunchArgument(
            'scene_color_compressed_topic',
            default_value='/camera_scene/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'scene_color_topic', default_value='/camera_scene/color/image_raw'),
        DeclareLaunchArgument(
            'scene_depth_topic',
            default_value='/camera_scene/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'left_wrist_color_topic',
            default_value='/camera_left_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'left_wrist_color_compressed_topic',
            default_value='/camera_left_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'left_wrist_depth_topic',
            default_value='/camera_left_wrist/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_topic',
            default_value='/camera_right_wrist/color/image_raw'),
        DeclareLaunchArgument(
            'right_wrist_color_compressed_topic',
            default_value='/camera_right_wrist/color/image_raw/compressed'),
        DeclareLaunchArgument(
            'right_wrist_depth_topic',
            default_value='/camera_right_wrist/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('can_left', default_value='can_left'),
        DeclareLaunchArgument('can_right', default_value='can_right'),
        DeclareLaunchArgument('speed_percent', default_value='5'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        DeclareLaunchArgument(
            'recover_joint_limits', default_value='true',
            description=(
                'Clear latched faults and trim joint4/joint7 back inside the '
                'deployment caps after the drivers start; motors stay enabled.'
            ),
        ),
        DeclareLaunchArgument(
            'recover_joint_limits_delay_sec', default_value='8.0',
            description='Delay before the joint-limit recovery step.'),
        DeclareLaunchArgument('table_width', default_value='1.80'),
        DeclareLaunchArgument('table_depth', default_value='0.90'),
        DeclareLaunchArgument('table_thickness', default_value='0.03'),
        DeclareLaunchArgument('base_spacing', default_value='0.60'),
        DeclareLaunchArgument('front_edge_offset', default_value='0.06'),
        DeclareLaunchArgument('left_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('right_base_yaw', default_value='3.141592653589793'),
        DeclareLaunchArgument('include_gripper', default_value='true'),
        webapp,
        drivers,
        grippers,
        cameras,
        moveit,
    ])
