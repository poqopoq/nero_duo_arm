#!/usr/bin/env python3
"""Capture the current left/right NERO joint states as a support pose."""
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

SIDES = ('left', 'right')
J4_INDEX = 3
J4_MAX = 2.14
ROOT = Path(os.environ.get('NERO_DUO_ROOT', Path(__file__).resolve().parents[2]))
OUTPUT = ROOT / 'deployment' / 'config' / 'support_pose.yaml'


class SupportPoseCapture(Node):
    def __init__(self):
        super().__init__('capture_support_pose')
        self.values = {side: None for side in SIDES}
        for side in SIDES:
            self.create_subscription(
                JointState,
                f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._cb(s, msg),
                10,
            )

    def _cb(self, side, msg):
        state = dict(zip(msg.name, msg.position))
        if all(f'joint{i}' in state for i in range(1, 8)):
            self.values[side] = [float(state[f'joint{i}']) for i in range(1, 8)]

    def ready(self):
        return all(value is not None for value in self.values.values())


rclpy.init()
node = SupportPoseCapture()
deadline = time.monotonic() + 10.0
while time.monotonic() < deadline and not node.ready() and rclpy.ok():
    rclpy.spin_once(node, timeout_sec=0.1)
if not node.ready():
    print('SUPPORT_CAPTURE_FAILED')
    node.destroy_node(); rclpy.shutdown(); raise SystemExit(2)

captured = {side: list(node.values[side]) for side in SIDES}
for side in SIDES:
    if captured[side][J4_INDEX] > J4_MAX:
        print(
            f'{side} joint4 feedback {captured[side][J4_INDEX]:.9f} exceeds '
            f'{J4_MAX:.2f}; clamping for the support pose')
        captured[side][J4_INDEX] = J4_MAX

OUTPUT.parent.mkdir(parents=True, exist_ok=True)
with OUTPUT.open('w', encoding='utf-8') as f:
    f.write('# Canonical support pose captured from the live NERO joint feedback.\n')
    f.write('# Joint 4 uses the deployment cap of 2.14 rad'
            ' (native upper bound: 2.146755).\n')
    f.write('support_pose:\n')
    for side in SIDES:
        for idx, value in enumerate(captured[side], start=1):
            f.write(f'  {side}_joint{idx}: {value:.9f}\n')
print('SUPPORT_POSE_SAVED', OUTPUT)
for side in SIDES:
    print(side, [round(v, 6) for v in captured[side]])
node.destroy_node()
rclpy.shutdown()
