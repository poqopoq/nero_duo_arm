#!/usr/bin/env python3
import os
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

SIDES = ('left', 'right')
TOLERANCE = 0.03
TIMEOUT = 10.0
RATE_HZ = 20.0
ROOT = Path(os.environ.get('NERO_DUO_ROOT', Path(__file__).resolve().parents[2]))
SUPPORT_FILE = ROOT / 'deployment' / 'config' / 'support_pose.yaml'


class ReturnToSupport(Node):
    def __init__(self):
        super().__init__('return_to_standard_support')
        self.current = {side: None for side in SIDES}
        self.enable_clients = {
            side: self.create_client(SetBool, f'/{side}/enable_agx_arm')
            for side in SIDES
        }
        self.gate_clients = {
            side: self.create_client(SetBool, f'/{side}/control_enable')
            for side in SIDES
        }
        self.stop_clients = {
            side: self.create_client(Empty, f'/{side}/emergency_stop')
            for side in SIDES
        }
        self.command_pubs = {
            side: self.create_publisher(JointState, f'/{side}/control/joint_states', 10)
            for side in SIDES
        }
        for side in SIDES:
            self.create_subscription(
                JointState, f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._feedback(s, msg), 10)

    def _feedback(self, side, msg):
        values = dict(zip(msg.name, msg.position))
        if all(f'joint{i}' in values for i in range(1, 8)):
            self.current[side] = [float(values[f'joint{i}']) for i in range(1, 8)]

    def spin_once(self, timeout=0.05):
        rclpy.spin_once(self, timeout_sec=timeout)

    def wait_feedback(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            self.spin_once()
            if all(v is not None for v in self.current.values()):
                return True
        return False

    def call_setbool(self, client, data):
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        req = SetBool.Request(); req.data = data
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        return future.done() and future.result() is not None

    def call_stop(self, side):
        client = self.stop_clients[side]
        if not client.wait_for_service(timeout_sec=3.0):
            return False
        future = client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.done() and future.result() is not None

    def close_gates(self):
        for side in SIDES:
            self.call_setbool(self.gate_clients[side], False)

    def publish_target(self, target):
        for side in SIDES:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'joint{i}' for i in range(1, 8)]
            msg.position = target[side]
            self.command_pubs[side].publish(msg)


data = yaml.safe_load(SUPPORT_FILE.read_text())['support_pose']
target = {
    side: [float(data[f'{side}_joint{i}']) for i in range(1, 8)]
    for side in SIDES
}

rclpy.init()
node = ReturnToSupport()
if not node.wait_feedback():
    print('FEEDBACK_TIMEOUT')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(2)

print('SUPPORT_TARGET', {s: [round(x, 6) for x in target[s]] for s in SIDES})
for side in SIDES:
    if not node.call_setbool(node.enable_clients[side], True):
        print(f'{side.upper()}_ENABLE_FAILED')
        node.close_gates(); node.destroy_node(); rclpy.shutdown(); raise SystemExit(3)
    if not node.call_setbool(node.gate_clients[side], True):
        print(f'{side.upper()}_GATE_OPEN_FAILED')
        node.close_gates();
        for s in SIDES: node.call_setbool(node.enable_clients[s], False)
        node.destroy_node(); rclpy.shutdown(); raise SystemExit(3)

deadline = time.monotonic() + TIMEOUT
success = False
max_error = float('inf')
while time.monotonic() < deadline and rclpy.ok():
    node.publish_target(target)
    node.spin_once(timeout=0.03)
    max_error = max(
        abs(actual - desired)
        for side in SIDES
        for actual, desired in zip(node.current[side], target[side])
    )
    if max_error <= TOLERANCE:
        success = True
        break
    time.sleep(1.0 / RATE_HZ)

# Hold the final target briefly so the controller/servo loop settles.
hold_end = time.monotonic() + 1.0
while time.monotonic() < hold_end and rclpy.ok():
    node.publish_target(target)
    node.spin_once(timeout=0.03)
    time.sleep(1.0 / RATE_HZ)
    max_error = max(
        abs(actual - desired)
        for side in SIDES
        for actual, desired in zip(node.current[side], target[side])
    )
    if max_error <= TOLERANCE:
        success = True

node.close_gates()
print('FINAL_POSE', {s: [round(x, 6) for x in node.current[s]] for s in SIDES})
print('MAX_SUPPORT_ERROR', round(max_error, 6))
print('AT_SUPPORT', success)
if success:
    for side in SIDES:
        print(f'{side.upper()}_EMERGENCY_STOP', node.call_stop(side))
    node.close_gates()
    for side in SIDES:
        print(f'{side.upper()}_DISABLE', node.call_setbool(node.enable_clients[side], False))
    exit_code = 0
else:
    for side in SIDES:
        print(f'{side.upper()}_EMERGENCY_STOP', node.call_stop(side))
    node.close_gates()
    print('MANUAL_SUPPORT_REQUIRED: motors remain enabled')
    exit_code = 4
node.destroy_node()
rclpy.shutdown()
raise SystemExit(exit_code)
