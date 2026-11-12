import ast
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
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


# link7 -> CTAG2F90D pinch point is the same transform the URDF applies through
# link7 -> tool0 -> tcp_link. The 213.3 mm tool-axis offset includes the 42.5 mm
# force-sensor/connector stack. Keep this in sync with the URDF defaults.
DEFAULT_TCP_OFFSET = '[0.2443, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
RECOVER_SCRIPT = _resolve_recover_script()


def _parse_tcp_offset(raw: str) -> list[float]:
    try:
        values = ast.literal_eval(raw)
    except (SyntaxError, ValueError) as exc:
        raise ValueError(f'Invalid tcp_offset {raw!r}: {exc}') from exc
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(
            f'tcp_offset must be [x, y, z, roll, pitch, yaw], got {raw!r}')
    return [float(value) for value in values]


def _nodes(context):
    common = {
        'arm_type': 'nero',
        'effector_type': 'none',
        'auto_enable': LaunchConfiguration('auto_enable'),
        'speed_percent': LaunchConfiguration('speed_percent'),
        'control_enabled': False,
        'pub_rate': 200,
        'tcp_offset': _parse_tcp_offset(
            LaunchConfiguration('tcp_offset').perform(context)),
    }

    return [
        Node(
            package='agx_arm_ctrl',
            executable='agx_arm_ctrl_single',
            namespace='left',
            name='agx_arm_ctrl_single_node',
            output='screen',
            parameters=[dict(
                common,
                can_port=LaunchConfiguration('can_left'),
            )],
        ),
        Node(
            package='agx_arm_ctrl',
            executable='agx_arm_ctrl_single',
            namespace='right',
            name='agx_arm_ctrl_single_node',
            output='screen',
            parameters=[dict(
                common,
                can_port=LaunchConfiguration('can_right'),
            )],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('can_left', default_value='can_left'),
        DeclareLaunchArgument('can_right', default_value='can_right'),
        DeclareLaunchArgument('speed_percent', default_value='5'),
        DeclareLaunchArgument('auto_enable', default_value='true'),
        DeclareLaunchArgument(
            'tcp_offset',
            default_value=DEFAULT_TCP_OFFSET,
            description=(
                'TCP offset from link7 as [x, y, z, roll, pitch, yaw] '
                'in metres/radians. Defaults to the CTAG2F90D pinch point.'
            ),
        ),
        DeclareLaunchArgument(
            'recover_joint_limits',
            default_value='true',
            description=(
                'After power-up, clear latched faults and trim joint4/joint7 '
                '(plus any slightly over-limit joint) back inside the caps. '
                'Runs a guarded low-speed move and keeps the motors enabled.'
            ),
        ),
        DeclareLaunchArgument(
            'recover_joint_limits_delay_sec',
            default_value='8.0',
            description='Delay before the joint-limit recovery step.',
        ),
        OpaqueFunction(function=_nodes),
        TimerAction(
            period=LaunchConfiguration('recover_joint_limits_delay_sec'),
            condition=IfCondition(LaunchConfiguration('recover_joint_limits')),
            actions=[ExecuteProcess(
                cmd=[str(RECOVER_SCRIPT), '--watch'],
                output='screen',
            )],
        ),
    ])
