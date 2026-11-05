#!/usr/bin/env python3
"""Low-speed joint motion plus software emergency-stop test."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

JOINT_NAMES = [f'joint{i}' for i in range(1, 8)]
SIDES = ('left', 'right')
DELTA = 0.15
MOVE_TIMEOUT_SECONDS = 2.0
MOTION_THRESHOLD = 0.01
POST_STOP_SECONDS = 2.5


class SoftEStopMotionTest(Node):
    def __init__(self, sides) -> None:
        self.sides = tuple(sides)
        super().__init__('soft_estop_motion_test')
        self.latest = {side: None for side in self.sides}
        self.pubs = {}
        self.stop_clients = {}
        self.gate_clients = {}
        self.enable_clients = {}
        self.was_enabled = set()
        for side in self.sides:
            self.create_subscription(
                JointState, f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._feedback_cb(s, msg), 10)
            self.pubs[side] = self.create_publisher(
                JointState, f'/{side}/control/joint_states', 10)
            self.stop_clients[side] = self.create_client(
                Empty, f'/{side}/emergency_stop')
            self.gate_clients[side] = self.create_client(
                SetBool, f'/{side}/control_enable')
            self.enable_clients[side] = self.create_client(
                SetBool, f'/{side}/enable_agx_arm')

    def _feedback_cb(self, side: str, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.latest[side] = [positions[name] for name in JOINT_NAMES]

    def wait_feedback(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.latest[side] is not None for side in self.sides):
                return True
        return False

    def wait_services(self, timeout: float = 3.0) -> bool:
        for side in self.sides:
            if not self.stop_clients[side].wait_for_service(timeout_sec=timeout):
                return False
            if not self.gate_clients[side].wait_for_service(timeout_sec=timeout):
                return False
        return True

    def call_setbool(self, client, data: bool, timeout: float = 4.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done() and future.result() is not None

    def call_stops(self, timeout: float = 4.0) -> bool:
        futures = {}
        for side in self.sides:
            if not self.stop_clients[side].wait_for_service(timeout_sec=timeout):
                return False
            futures[side] = self.stop_clients[side].call_async(Empty.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if all(f.done() for f in futures.values()):
                break
        return all(f.done() and f.result() is not None for f in futures.values())

    def publish(self, side: str, positions) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pubs[side].publish(msg)

    def hold(self, positions_by_side, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            for side in self.sides:
                self.publish(side, positions_by_side[side])
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--namespace', choices=('left', 'right'))
    group.add_argument('--both', action='store_true')
    args = parser.parse_args()
    sides = SIDES if args.both else (args.namespace,)

    rclpy.init()
    node = SoftEStopMotionTest(sides)
    result = 1
    try:
        if not node.wait_feedback():
            print(f'[FAIL] 未收到完整关节反馈: {", ".join(sides)}', flush=True)
            return 2
        if not node.wait_services():
            print('[FAIL] 急停或控制门服务不可用，取消运动', flush=True)
            return 3

        q_start = {side: list(node.latest[side]) for side in sides}
        node.hold(q_start, 1.0)
        if any(node.latest[side] is None for side in sides):
            print('[FAIL] 预热期间反馈丢失', flush=True)
            return 5
        q_start = {side: list(node.latest[side]) for side in sides}

        for side in sides:
            if not node.call_setbool(node.gate_clients[side], True):
                print(f'[FAIL] 无法打开 {side} 控制门', flush=True)
                return 6
            node.was_enabled.add(side)

        target = {side: list(q_start[side]) for side in sides}
        for side in sides:
            target[side][6] += DELTA
            print(
                f'[TEST] {side} joint7 start={q_start[side][6]:.6f} '
                f'target={target[side][6]:.6f}', flush=True)

        motion_detected = {side: False for side in sides}
        deadline = time.monotonic() + MOVE_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            for side in sides:
                node.publish(side, target[side])
            rclpy.spin_once(node, timeout_sec=0.01)
            for side in sides:
                if (node.latest[side] is not None and
                        abs(node.latest[side][6] - q_start[side][6]) >= MOTION_THRESHOLD):
                    motion_detected[side] = True
            if all(motion_detected.values()):
                break

        q_at_stop = {
            side: list(node.latest[side]) if node.latest[side] is not None
            else target[side]
            for side in sides
        }
        stop_ok = node.call_stops()
        if stop_ok:
            for side in sides:
                node.call_setbool(node.gate_clients[side], False)
        stop_time = time.monotonic()
        for side in sides:
            print(
                f'[TEST] {side} motion_detected={motion_detected[side]} '
                f'joint7_at_stop={q_at_stop[side][6]:.6f}', flush=True)
        print(f'[TEST] software_stop_ok={stop_ok}', flush=True)

        samples = {side: [] for side in sides}
        while rclpy.ok() and time.monotonic() - stop_time < POST_STOP_SECONDS:
            rclpy.spin_once(node, timeout_sec=0.01)
            elapsed = time.monotonic() - stop_time
            for side in sides:
                if node.latest[side] is not None:
                    samples[side].append((elapsed, node.latest[side][6]))

        for side in sides:
            node.call_setbool(node.enable_clients[side], False)
            node.was_enabled.discard(side)

        passed = stop_ok
        for side in sides:
            q_final = (
                list(node.latest[side])
                if node.latest[side] is not None else q_at_stop[side]
            )
            moved = abs(q_at_stop[side][6] - q_start[side][6])
            late = [v for t, v in samples[side] if t >= 0.5]
            late_drift = max(late) - min(late) if late else math.inf
            post_stop_change = abs(q_final[6] - q_at_stop[side][6])
            print(
                f'[RESULT] {side} moved_before_stop={moved:.6f} rad '
                f'post_stop_change={post_stop_change:.6f} rad '
                f'late_window_drift={late_drift:.6f} rad '
                f'joint7_final={q_final[6]:.6f}', flush=True)
            if not (motion_detected[side] and moved >= 0.01 and
                    late_drift <= 0.01):
                passed = False

        if passed:
            print('[PASS] 低速运动发生，软件急停后关节稳定', flush=True)
            result = 0
        else:
            print('[FAIL] 运动或软件急停效果未达到验收阈值', flush=True)
    finally:
        if node.was_enabled:
            node.call_stops()
        for side in list(node.was_enabled):
            node.call_setbool(node.gate_clients[side], False)
            node.call_setbool(node.enable_clients[side], False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
