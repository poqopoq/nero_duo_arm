#!/usr/bin/env python3
"""Single-joint low-speed motion plus software emergency-stop test."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

JOINT_NAMES = [f'joint{i}' for i in range(1, 8)]
DELTA = 0.15
MOVE_TIMEOUT_SECONDS = 2.0
MOTION_THRESHOLD = 0.01
POST_STOP_SECONDS = 2.5


class SoftEStopMotionTest(Node):
    def __init__(self, namespace: str) -> None:
        self.namespace = namespace.strip('/')
        super().__init__(f'{self.namespace}_soft_estop_motion_test')
        self.latest = None
        self.create_subscription(
            JointState, f'/{self.namespace}/feedback/joint_states',
            self._feedback_cb, 10)
        self.pub = self.create_publisher(
            JointState, f'/{self.namespace}/control/joint_states', 10)
        self.stop_client = self.create_client(
            Empty, f'/{self.namespace}/emergency_stop')
        self.gate_client = self.create_client(
            SetBool, f'/{self.namespace}/control_enable')
        self.enable_client = self.create_client(
            SetBool, f'/{self.namespace}/enable_agx_arm')
        self.was_enabled = False

    def _feedback_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        if all(name in positions for name in JOINT_NAMES):
            self.latest = [positions[name] for name in JOINT_NAMES]

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)

    def wait_feedback(self, timeout: float = 10.0) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None:
                return True
        return False

    def call_setbool(self, client, data: bool, timeout: float = 4.0) -> bool:
        if not client.wait_for_service(timeout_sec=timeout):
            return False
        req = SetBool.Request()
        req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done() and future.result() is not None

    def call_stop(self, timeout: float = 4.0) -> bool:
        if not self.stop_client.wait_for_service(timeout_sec=timeout):
            return False
        future = self.stop_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return future.done() and future.result() is not None

    def publish(self, positions) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = positions
        self.pub.publish(msg)

    def hold(self, positions, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish(positions)
            rclpy.spin_once(self, timeout_sec=0.02)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--namespace', choices=('left', 'right'), required=True)
    args = parser.parse_args()

    rclpy.init()
    node = SoftEStopMotionTest(args.namespace)
    result = 1
    try:
        if not node.wait_feedback():
            print('[FAIL] 未收到完整关节反馈', flush=True)
            return 2
        if not node.stop_client.wait_for_service(timeout_sec=3.0):
            print(f'[FAIL] /{args.namespace}/emergency_stop 服务不可用，取消运动', flush=True)
            return 3
        if not node.gate_client.wait_for_service(timeout_sec=3.0):
            print(f'[FAIL] /{args.namespace}/control_enable 服务不可用，取消运动', flush=True)
            return 4

        q_start = list(node.latest)
        node.hold(q_start, 1.0)
        if node.latest is None:
            print('[FAIL] 预热期间反馈丢失', flush=True)
            return 5
        q_start = list(node.latest)

        if not node.call_setbool(node.gate_client, True):
            print('[FAIL] 无法打开控制门', flush=True)
            return 6
        node.was_enabled = True

        target = list(q_start)
        target[6] += DELTA
        print(f'[TEST] joint7 start={q_start[6]:.6f} target={target[6]:.6f}', flush=True)

        motion_detected = False
        deadline = time.monotonic() + MOVE_TIMEOUT_SECONDS
        while rclpy.ok() and time.monotonic() < deadline:
            node.publish(target)
            rclpy.spin_once(node, timeout_sec=0.01)
            if (node.latest is not None and
                    abs(node.latest[6] - q_start[6]) >= MOTION_THRESHOLD):
                motion_detected = True
                break

        q_at_stop = list(node.latest) if node.latest is not None else target
        stop_ok = node.call_stop()
        if stop_ok:
            node.call_setbool(node.gate_client, False)
        stop_time = time.monotonic()
        print(
            f'[TEST] motion_detected={motion_detected} software_stop_ok={stop_ok} '
            f'joint7_at_stop={q_at_stop[6]:.6f}',
            flush=True,
        )

        samples = []
        while rclpy.ok() and time.monotonic() - stop_time < POST_STOP_SECONDS:
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.latest is not None:
                samples.append((time.monotonic() - stop_time, node.latest[6]))

        node.call_setbool(node.enable_client, False)
        node.was_enabled = False

        q_final = list(node.latest) if node.latest is not None else q_at_stop
        moved = abs(q_at_stop[6] - q_start[6])
        late = [v for t, v in samples if t >= 0.5]
        late_drift = max(late) - min(late) if late else math.inf
        post_stop_change = abs(q_final[6] - q_at_stop[6])
        print(
            f'[RESULT] moved_before_stop={moved:.6f} rad '
            f'post_stop_change={post_stop_change:.6f} rad '
            f'late_window_drift={late_drift:.6f} rad '
            f'joint7_final={q_final[6]:.6f}',
            flush=True,
        )
        if motion_detected and moved >= 0.01 and stop_ok and late_drift <= 0.01:
            print('[PASS] 低速运动发生，软件急停后关节稳定', flush=True)
            result = 0
        else:
            print('[FAIL] 运动或软件急停效果未达到验收阈值', flush=True)
    finally:
        if node.was_enabled:
            node.call_stop()
            node.call_setbool(node.gate_client, False)
            node.call_setbool(node.enable_client, False)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
