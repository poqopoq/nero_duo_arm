#!/usr/bin/python3
"""Compare driver TCP feedback with MoveIt FK for the deployed NERO arms."""

from __future__ import annotations

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header


def _position_delta(a: PoseStamped, b: PoseStamped) -> float:
    return math.sqrt(
        (a.pose.position.x - b.pose.position.x) ** 2
        + (a.pose.position.y - b.pose.position.y) ** 2
        + (a.pose.position.z - b.pose.position.z) ** 2
    )


def _quaternion_angle(a: PoseStamped, b: PoseStamped) -> float:
    qa = a.pose.orientation
    qb = b.pose.orientation
    dot = abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w)
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


class ContractChecker(Node):
    def __init__(self, timeout: float) -> None:
        super().__init__('nero_tcp_contract_check')
        self.timeout = timeout
        self.latest: dict[str, PoseStamped] = {}
        self.joints: dict[str, list[float]] = {}
        for side in ('left', 'right'):
            self.create_subscription(
                PoseStamped,
                f'/{side}/feedback/tcp_pose',
                lambda msg, side=side: self.latest.__setitem__(side, msg),
                10,
            )
            self.create_subscription(
                JointState,
                f'/{side}/feedback/joint_states',
                lambda msg, side=side: self.joints.__setitem__(
                    side, [float(value) for value in msg.position]),
                10,
            )
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        if not self.fk_client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError('no /compute_fk service; start MoveIt first')

    def wait_for_data(self) -> None:
        deadline = self.get_clock().now().nanoseconds + int(self.timeout * 1e9)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                all(side in self.latest for side in ('left', 'right'))
                and all(len(self.joints.get(side, [])) >= 7 for side in ('left', 'right'))
            ):
                return
            if self.get_clock().now().nanoseconds >= deadline:
                raise RuntimeError(
                    'timed out waiting for /feedback/tcp_pose and '
                    '/feedback/joint_states on both arms')

    def compute_fk(self, side: str) -> PoseStamped:
        request = GetPositionFK.Request()
        request.header = Header(frame_id=f'{side}_base_link')
        request.fk_link_names = [f'{side}_tcp_link']
        request.robot_state = RobotState()
        names: list[str] = []
        positions: list[float] = []
        for arm in ('left', 'right'):
            names.extend(f'{arm}_joint{index}' for index in range(1, 8))
            positions.extend(self.joints[arm][:7])
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        future = self.fk_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        response = future.result()
        if response is None:
            raise RuntimeError(f'{side} compute_fk returned no response')
        if response.error_code.val != 1 or not response.pose_stamped:
            raise RuntimeError(
                f'{side} compute_fk failed with code {response.error_code.val}')
        return response.pose_stamped[0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=5.0)
    parser.add_argument('--position-tolerance', type=float, default=0.001)
    parser.add_argument('--rotation-tolerance', type=float, default=0.01)
    args = parser.parse_args()

    rclpy.init()
    node = ContractChecker(args.timeout)
    try:
        node.wait_for_data()
        failed = False
        for side in ('left', 'right'):
            expected = node.compute_fk(side)
            actual = node.latest[side]
            position_error = _position_delta(actual, expected)
            rotation_error = _quaternion_angle(actual, expected)
            ok = (
                position_error <= args.position_tolerance
                and rotation_error <= args.rotation_tolerance
            )
            failed |= not ok
            print(
                f'{side}: tcp_pose=({actual.pose.position.x:+.4f}, '
                f'{actual.pose.position.y:+.4f}, {actual.pose.position.z:+.4f}) '
                f'fk=({expected.pose.position.x:+.4f}, '
                f'{expected.pose.position.y:+.4f}, '
                f'{expected.pose.position.z:+.4f}) '
                f'position_error={position_error * 1000:.3f} mm '
                f'rotation_error={rotation_error:.5f} rad '
                f'{"OK" if ok else "MISMATCH"}'
            )
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
