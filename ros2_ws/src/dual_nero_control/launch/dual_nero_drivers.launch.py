import ast
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# link7 -> CTAG2F90D pinch point is the same transform the URDF applies through
# link7 -> tool0 -> tcp_link. Keep this in sync with two_nero_tabletop.urdf.xacro.
DEFAULT_TCP_OFFSET = '[0.2018, 0.0, -0.0235, -1.5708, 0.0, -1.5708]'
RECOVER_SCRIPT = (
    Path(__file__).resolve().parents[4]
    / 'deployment' / 'scripts' / 'recover_joint_limits.sh')


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
            default_value='false',
            description=(
                'After power-up, trim joint4 back inside the deployment cap '
                'and keep the motors enabled. Runs a guarded low-speed move.'
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
                cmd=[str(RECOVER_SCRIPT)],
                output='screen',
            )],
        ),
    ])
