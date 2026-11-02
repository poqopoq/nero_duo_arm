#!/usr/bin/env python3
"""Validate linked low-speed software E-stop for both NERO arms."""
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


class DualSoftEStopTest(Node):
    def __init__(self) -> None:
        super().__init__('dual_soft_estop_motion_test')
        self.latest = {side: None for side in SIDES}
        self.pubs = {}
        self.stop_clients = {}
        self.gate_clients = {}
        self.enable_clients = {}
        self.was_enabled = False
        for side in SIDES:
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

    def _feedback_cb(self, side, msg) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.latest[side] = [positions[name] for name in JOINT_NAMES]

    def wait_feedback(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(self.latest[s] is not None for s in SIDES):
                return True
        return False

    def wait_services(self, timeout: float = 4.0) -> bool:
        for side in SIDES:
            if not self.stop_clients[side].wait_for_service(timeout):
                return False
            if not self.gate_clients[side].wait_for_service(timeout):
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

    def call_both_stops(self, timeout: float = 4.0):
        futures = {}
        for side in SIDES:
            futures[side] = self.stop_clients[side].call_async(Empty.Request())
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            if all(f.done() for f in futures.values()):
                break
        return {
            side: f.done() and f.result() is not None
            for side, f in futures.items()
        }

    def publish(self, side, positions) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pubs[side].publish(msg)

    def hold(self, positions, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            for side in SIDES:
                self.publish(side, positions[side])
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    rclpy.init()
    node = DualSoftEStopTest()
    result = 1
    try:
        if not node.wait_feedback():
            print('[FAIL] 未收到两侧完整反馈', flush=True)
            return 2
        if not node.wait_services():
            print('[FAIL] 急停或控制门服务不可用，取消运动', flush=True)
            return 3

        q_start = {s: list(node.latest[s]) for s in SIDES}
        node.hold(q_start, 1.0)
        q_start = {s: list(node.latest[s]) for s in SIDES}
        for side in SIDES:
            if not node.call_setbool(node.gate_clients[side], True):
                print(f'[FAIL] 无法打开 {side} 控制门', flush=True)
                return 4
        node.was_enabled = True

        targets = {s: list(q_start[s]) for s in SIDES}
        for side in SIDES:
            targets[side][6] += DELTA
        print(
            '[TEST] joint7 start/target ' +
            ' '.join(
                f'{s}={q_start[s][6]:.6f}/{targets[s][6]:.6f}'
                for s in SIDES),
            flush=True,
        )

        moved = {s: False for s in SIDES}
        deadline = time.monotonic() + MOVE_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            for side in SIDES:
                node.publish(side, targets[side])
            rclpy.spin_once(node, timeout_sec=0.01)
            for side in SIDES:
                if (node.latest[side] is not None and
                        abs(node.latest[side][6] - q_start[side][6]) >= MOTION_THRESHOLD):
                    moved[side] = True
            if all(moved.values()):
                break

        q_at_stop = {s: list(node.latest[s]) for s in SIDES}
        stops = node.call_both_stops()
        for side in SIDES:
            if stops[side]:
                node.call_setbool(node.gate_clients[side], False)
        stop_time = time.monotonic()
        print(f'[TEST] motion_detected={moved} stop_ok={stops}', flush=True)

        samples = {s: [] for s in SIDES}
        while rclpy.ok() and time.monotonic() - stop_time < POST_STOP_SECONDS:
            rclpy.spin_once(node, timeout_sec=0.01)
            elapsed = time.monotonic() - stop_time
            for side in SIDES:
                if node.latest[side] is not None:
                    samples[side].append((elapsed, node.latest[side][6]))

        for side in SIDES:
            node.call_setbool(node.enable_clients[side], False)
        node.was_enabled = False

        all_ok = True
        for side in SIDES:
            q_final = node.latest[side][6]
            pre_stop = abs(q_at_stop[side][6] - q_start[side][6])
            post_stop = abs(q_final - q_at_stop[side][6])
            late = [v for t, v in samples[side] if t >= 0.5]
            late_drift = max(late) - min(late) if late else float('inf')
            ok = moved[side] and stops[side] and pre_stop >= MOTION_THRESHOLD and late_drift <= 0.01
            all_ok = all_ok and ok
            print(
                f'[RESULT] {side}: moved_before_stop={pre_stop:.6f} '
                f'post_stop_change={post_stop:.6f} '
                f'late_window_drift={late_drift:.6f} final={q_final:.6f} '
                f'ok={ok}',
                flush=True,
            )
        if all_ok:
            print('[PASS] 双臂低速运动后联动软件急停通过', flush=True)
            result = 0
        else:
            print('[FAIL] 双臂联动软件急停未达到验收阈值', flush=True)
    finally:
        if node.was_enabled:
            node.call_both_stops()
            for side in SIDES:
                node.call_setbool(node.gate_clients[side], False)
                node.call_setbool(node.enable_clients[side], False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
