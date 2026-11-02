#!/usr/bin/env python3
"""MoveIt FollowJointTrajectory bridge for two namespaced NERO drivers.

dry_run=true (default): interpolate the trajectory and publish simulated
joint states only. No command is sent to the robot drivers.
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, SetBool

SIDES = ('left', 'right')
JOINTS = {
    side: [f'{side}_joint{i}' for i in range(1, 8)]
    for side in SIDES
}
ALL_JOINTS = JOINTS['left'] + JOINTS['right']


def _duration_to_sec(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def _stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class DualTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__('dual_nero_trajectory_bridge')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('startup_feedback_timeout', 5.0)
        self.declare_parameter('final_hold_sec', 0.5)

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.feedback_timeout = float(self.get_parameter('startup_feedback_timeout').value)
        self.final_hold_sec = float(self.get_parameter('final_hold_sec').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.feedback: Dict[str, Optional[List[float]]] = {s: None for s in SIDES}
        self.feedback_stamp: Dict[str, float] = {s: 0.0 for s in SIDES}
        self.sim_positions: Dict[str, List[float]] = {s: [0.0] * 7 for s in SIDES}
        self.command_pubs = {
            s: self.create_publisher(
                JointState, f'/{s}/control/joint_states', qos)
            for s in SIDES
        }
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', qos)
        self.gate_clients = {
            s: self.create_client(SetBool, f'/{s}/control_enable')
            for s in SIDES
        }
        self.stop_clients = {
            s: self.create_client(Empty, f'/{s}/emergency_stop')
            for s in SIDES
        }

        for side in SIDES:
            self.create_subscription(
                JointState,
                f'/{side}/feedback/joint_states',
                lambda msg, s=side: self._feedback_cb(s, msg),
                qos,
            )

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/dual_arm_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.create_timer(0.02, self._publish_joint_states)
        self.get_logger().info(
            f'Trajectory bridge ready; dry_run={self.dry_run}, rate={self.rate_hz} Hz')

    def _feedback_cb(self, side: str, msg: JointState) -> None:
        positions = {name: value for name, value in zip(msg.name, msg.position)}
        local = [positions.get(f'joint{i}') for i in range(1, 8)]
        if all(value is not None for value in local):
            self.feedback[side] = [float(value) for value in local]
            self.feedback_stamp[side] = time.monotonic()

    def _publish_joint_states(self) -> None:
        names: List[str] = []
        positions: List[float] = []
        for side in SIDES:
            side_positions = (
                self.sim_positions[side] if self.dry_run else self.feedback[side])
            if side_positions is None:
                continue
            names.extend(JOINTS[side])
            positions.extend(side_positions)
        if not names:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = positions
        self.joint_state_pub.publish(msg)

    def _current_side(self, side: str) -> List[float]:
        if self.feedback[side] is not None:
            return list(self.feedback[side])
        return [0.0] * 7

    def _side_from_joint(self, name: str) -> Optional[str]:
        for side in SIDES:
            if name in JOINTS[side]:
                return side
        return None

    def goal_callback(self, goal_request) -> GoalResponse:
        names = list(goal_request.trajectory.joint_names)
        if not names or any(name not in ALL_JOINTS for name in names):
            self.get_logger().warn('Rejecting trajectory with missing or unknown joints')
            return GoalResponse.REJECT
        points = goal_request.trajectory.points
        if not points:
            self.get_logger().warn('Rejecting empty trajectory')
            return GoalResponse.REJECT
        previous = -math.inf
        for point in points:
            t = _duration_to_sec(point.time_from_start)
            if t < previous or len(point.positions) != len(names):
                self.get_logger().warn('Rejecting malformed trajectory points')
                return GoalResponse.REJECT
            previous = t
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.01)
        return future.done()

    def _call_setbool(self, side: str, data: bool) -> bool:
        client = self.gate_clients[side]
        if not client.wait_for_service(timeout_sec=self.feedback_timeout):
            self.get_logger().error(f'{side} control_enable service unavailable')
            return False
        request = SetBool.Request()
        request.data = data
        future = client.call_async(request)
        return self._wait_future(future, self.feedback_timeout) and future.result() is not None

    def _emergency_stop(self, sides) -> None:
        if self.dry_run:
            return
        for side in sides:
            client = self.stop_clients[side]
            if client.wait_for_service(timeout_sec=0.5):
                future = client.call_async(Empty.Request())
                self._wait_future(future, 1.0)
            self._call_setbool(side, False)

    def _wait_feedback(self, sides, timeout: float) -> bool:
        if self.dry_run:
            return True
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if all(self.feedback[s] is not None for s in sides):
                return True
            time.sleep(0.05)
        return False

    def _publish_command(self, side: str, positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i}' for i in range(1, 8)]
        msg.position = positions
        self.command_pubs[side].publish(msg)

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        names = list(trajectory.joint_names)
        points = trajectory.points
        involved = sorted({self._side_from_joint(n) for n in names})
        result = FollowJointTrajectory.Result()

        if not self._wait_feedback(involved, self.feedback_timeout):
            self.get_logger().error('Required driver feedback unavailable')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        if not self.dry_run:
            for side in involved:
                if not self._call_setbool(side, True):
                    self._emergency_stop(involved)
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    goal_handle.abort()
                    return result

        starts = {side: self._current_side(side) for side in involved}
        start_time = time.monotonic()
        end_time = _duration_to_sec(points[-1].time_from_start)
        period = 1.0 / self.rate_hz
        last_positions = {side: list(starts[side]) for side in involved}

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn(
                        f'Trajectory cancelled; dry_run={self.dry_run}')
                    self._emergency_stop(involved)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    return result

                elapsed = time.monotonic() - start_time
                t = min(elapsed, end_time)
                side_targets = {}
                for side in involved:
                    side_targets[side] = list(starts[side])

                # Interpolate every trajectory point into full seven-joint side states.
                previous_t = 0.0
                previous_values = {side: list(starts[side]) for side in involved}
                for point in points:
                    point_t = _duration_to_sec(point.time_from_start)
                    values = {side: list(previous_values[side]) for side in involved}
                    for name, value in zip(names, point.positions):
                        side = self._side_from_joint(name)
                        if side in values:
                            values[side][JOINTS[side].index(name)] = float(value)
                    if t <= point_t:
                        alpha = 0.0 if point_t <= previous_t else (t - previous_t) / (point_t - previous_t)
                        for side in involved:
                            side_targets[side] = [
                                previous_values[side][i] + alpha * (values[side][i] - previous_values[side][i])
                                for i in range(7)
                            ]
                        break
                    previous_t = point_t
                    previous_values = values

                for side in involved:
                    last_positions[side] = side_targets[side]
                    if self.dry_run:
                        self.sim_positions[side] = list(side_targets[side])
                    else:
                        self._publish_command(side, side_targets[side])

                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = names
                feedback.actual.positions = [
                    last_positions[self._side_from_joint(name)][
                        JOINTS[self._side_from_joint(name)].index(name)]
                    for name in names
                ]
                sec = int(elapsed)
                feedback.actual.time_from_start = Duration(
                    sec=sec, nanosec=int((elapsed - sec) * 1e9))
                goal_handle.publish_feedback(feedback)

                if elapsed >= end_time:
                    break
                time.sleep(period)

            if self.final_hold_sec > 0.0:
                hold_deadline = time.monotonic() + self.final_hold_sec
                while rclpy.ok() and time.monotonic() < hold_deadline:
                    for side in involved:
                        if self.dry_run:
                            self.sim_positions[side] = list(last_positions[side])
                        else:
                            self._publish_command(side, last_positions[side])
                    time.sleep(period)

            if not self.dry_run:
                for side in involved:
                    self._call_setbool(side, False)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self.get_logger().info(
                f'Trajectory completed in {end_time:.3f}s; dry_run={self.dry_run}')
            return result
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Trajectory execution failed: {exc}')
            if not self.dry_run:
                self._emergency_stop(involved)
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DualTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
