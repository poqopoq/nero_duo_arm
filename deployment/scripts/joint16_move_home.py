#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

NAMES = [
    *(f'left_joint{index}' for index in range(1, 8)),
    *(f'right_joint{index}' for index in range(1, 8)),
]
HOME = [
    -0.07127925, -1.36069359, 0.02806489, 1.88160456,
    0.04045673, 0.00094248, 1.57072651,
]


class HomeMover(Node):
    def __init__(self) -> None:
        super().__init__('joint16_move_home')
        self.current: dict[str, float] | None = None
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, 10)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')

    def _on_joint_state(self, message: JointState) -> None:
        values = dict(zip(message.name, message.position))
        if all(name in values for name in NAMES):
            self.current = {name: float(values[name]) for name in NAMES}

    def wait_current(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current is not None:
                return True
        return False

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def execute_home(self, timeout: float) -> int:
        goal = MoveGroup.Goal()
        goal.request = MotionPlanRequest()
        goal.request.group_name = 'both_arms'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.05
        goal.request.max_acceleration_scaling_factor = 0.05
        goal.request.start_state.is_diff = True
        constraints = Constraints()
        for name, value in zip(NAMES, HOME + HOME):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.look_around = False

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print('HOME_GOAL_REJECTED')
            return 3

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        result = result_future.result()
        if result is None:
            print('HOME_NO_RESULT')
            return 4
        print(f'HOME_ERROR_CODE {result.result.error_code.val}')
        return 0 if result.result.error_code.val == 1 else 5


def main() -> int:
    parser = argparse.ArgumentParser(description='Move both NERO arms to the joint16 training home')
    parser.add_argument('--yes', action='store_true', help='skip the interactive MOVE confirmation')
    parser.add_argument('--timeout', type=float, default=120.0, help='MoveIt result timeout in seconds')
    args = parser.parse_args()

    rclpy.init()
    node = HomeMover()
    try:
        if not node.wait_current(15.0):
            print('JOINT_STATE_TIMEOUT')
            return 2
        if not node.move_client.wait_for_server(timeout_sec=15.0):
            print('MOVE_ACTION_UNAVAILABLE')
            return 3

        current = [node.current[name] for name in NAMES]
        targets = HOME + HOME
        max_delta = max(abs(value - target) for value, target in zip(current, targets))
        print('CURRENT', [round(value, 6) for value in current])
        print('MAX_HOME_DELTA_RAD', round(max_delta, 6))
        print('TEACHING_HOME', HOME)

        if not args.yes:
            answer = input('确认工作区无人、急停可用，输入 MOVE 继续: ')
            if answer.strip() != 'MOVE':
                print('CANCELLED')
                return 1

        code = node.execute_home(args.timeout)
        if code != 0:
            return code

        node.current = None
        node.spin_for(1.0)
        if not node.wait_current(5.0):
            print('FINAL_JOINT_STATE_TIMEOUT')
            return 6
        final = [node.current[name] for name in NAMES]
        max_error = max(abs(value - target) for value, target in zip(final, targets))
        print('FINAL', [round(value, 6) for value in final])
        print('MAX_HOME_ERROR_RAD', round(max_error, 6))
        return 0 if max_error <= 0.02 and math.isfinite(max_error) else 7
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
