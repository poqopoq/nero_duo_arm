#!/usr/bin/env python3
"""MoveIt FollowJointTrajectory bridge for two namespaced NERO drivers.

dry_run=true (default): interpolate the trajectory and publish simulated
joint states only. No command is sent to the robot drivers.
"""
from __future__ import annotations

import math
import threading
import time
from bisect import bisect_right
from typing import Dict, List, Optional

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


def _build_side_timeline(side, names, points, start):
    times = []
    positions = []
    velocities = []
    has_velocities = True
    previous_positions = list(start)
    previous_velocities = [0.0] * 7
    if points and _duration_to_sec(points[0].time_from_start) > 1e-9:
        times.append(0.0)
        positions.append(previous_positions)
        velocities.append(previous_velocities)
    for point in points:
        side_positions = list(previous_positions)
        side_velocities = list(previous_velocities)
        for name, value in zip(names, point.positions):
            if name in JOINTS[side]:
                side_positions[JOINTS[side].index(name)] = float(value)
        if len(point.velocities) == len(names):
            for name, value in zip(names, point.velocities):
                if name in JOINTS[side]:
                    side_velocities[JOINTS[side].index(name)] = float(value)
        else:
            has_velocities = False
        times.append(_duration_to_sec(point.time_from_start))
        positions.append(side_positions)
        velocities.append(side_velocities)
        previous_positions = side_positions
        previous_velocities = side_velocities
    return times, positions, velocities, has_velocities


def _interpolate_side(timeline, t, cubic):
    times, positions, velocities, has_velocities = timeline
    if not times:
        return [0.0] * 7
    if t <= times[0]:
        return list(positions[0])
    if t >= times[-1]:
        return list(positions[-1])

    index = min(max(bisect_right(times, t) - 1, 0), len(times) - 2)
    t0 = times[index]
    t1 = times[index + 1]
    if t1 <= t0:
        return list(positions[index + 1])

    alpha = (t - t0) / (t1 - t0)
    if not cubic or not has_velocities:
        return [
            positions[index][i]
            + alpha * (positions[index + 1][i] - positions[index][i])
            for i in range(7)
        ]

    alpha2 = alpha * alpha
    alpha3 = alpha2 * alpha
    h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0
    h10 = alpha3 - 2.0 * alpha2 + alpha
    h01 = -2.0 * alpha3 + 3.0 * alpha2
    h11 = alpha3 - alpha2
    duration = t1 - t0
    return [
        h00 * positions[index][i]
        + h10 * duration * velocities[index][i]
        + h01 * positions[index + 1][i]
        + h11 * duration * velocities[index + 1][i]
        for i in range(7)
    ]


class DualTrajectoryBridge(Node):
    def __init__(self) -> None:
        super().__init__('dual_nero_trajectory_bridge')

        self.declare_parameter('dry_run', True)
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('interpolation', 'linear')
        self.declare_parameter('startup_feedback_timeout', 5.0)
        self.declare_parameter('final_hold_sec', 0.5)
        self.declare_parameter('settle_timeout_sec', 1.0)
        self.declare_parameter('settle_tolerance_rad', 0.01)
        self.declare_parameter(
            'cancel_service_name', '/dual_nero_trajectory_bridge/cancel_motion')
        self.declare_parameter(
            'clear_cancel_service_name', '/dual_nero_trajectory_bridge/clear_cancel')

        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.rate_hz = float(self.get_parameter('command_rate_hz').value)
        self.interpolation = str(self.get_parameter('interpolation').value).lower()
        if self.interpolation not in ('linear', 'cubic'):
            self.get_logger().warn(
                f'Unknown interpolation {self.interpolation!r}; using linear')
            self.interpolation = 'linear'
        self.feedback_timeout = float(self.get_parameter('startup_feedback_timeout').value)
        self.final_hold_sec = float(self.get_parameter('final_hold_sec').value)
        self.settle_timeout_sec = max(
            0.0, float(self.get_parameter('settle_timeout_sec').value))
        self.settle_tolerance_rad = max(
            0.0, float(self.get_parameter('settle_tolerance_rad').value))
        self._cancel_lock = threading.Lock()
        self._cancel_requested = False

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
        self.create_service(
            Empty,
            str(self.get_parameter('cancel_service_name').value),
            self._cancel_motion_callback,
        )
        self.create_service(
            Empty,
            str(self.get_parameter('clear_cancel_service_name').value),
            self._clear_motion_cancel_callback,
        )
        self.create_timer(0.02, self._publish_joint_states)
        self.get_logger().info(
            f'Trajectory bridge ready; dry_run={self.dry_run}, '
            f'rate={self.rate_hz} Hz, interpolation={self.interpolation}')

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

    def _signed_tracking_errors(self, sides, targets) -> Dict[str, List[float]]:
        errors = {}
        for side in sides:
            if self.feedback[side] is not None:
                errors[side] = [
                    float(actual) - float(target)
                    for actual, target in zip(self.feedback[side], targets[side])
                ]
        return errors

    def _tracking_errors(self, sides, targets) -> Dict[str, List[float]]:
        return {
            side: [abs(error) for error in errors]
            for side, errors in self._signed_tracking_errors(sides, targets).items()
        }

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

    def _cancel_motion_callback(self, request, response):
        with self._cancel_lock:
            self._cancel_requested = True
        self.get_logger().warn('Trajectory cancellation requested')
        return response

    def _clear_motion_cancel_callback(self, request, response):
        with self._cancel_lock:
            self._cancel_requested = False
        self.get_logger().info('Trajectory cancellation latch cleared')
        return response

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
        with self._cancel_lock:
            cancel_requested = self._cancel_requested
        if cancel_requested:
            self.get_logger().warn('Rejecting trajectory while cancellation is latched')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PREEMPTED
            return result

        if not self._wait_feedback(involved, self.feedback_timeout):
            self.get_logger().error('Required driver feedback unavailable')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.abort()
            return result

        with self._cancel_lock:
            cancel_requested = self._cancel_requested
        if cancel_requested:
            self.get_logger().warn('Trajectory cancelled before gate open')
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PREEMPTED
            return result

        if not self.dry_run:
            for side in involved:
                if not self._call_setbool(side, True):
                    self._emergency_stop(involved)
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    goal_handle.abort()
                    return result
            with self._cancel_lock:
                cancel_requested = self._cancel_requested
            if cancel_requested:
                self.get_logger().warn('Trajectory cancelled after gate open')
                self._emergency_stop(involved)
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PREEMPTED
                return result

        starts = {side: self._current_side(side) for side in involved}
        timelines = {
            side: _build_side_timeline(side, names, points, starts[side])
            for side in involved
        }
        start_time = time.monotonic()
        end_time = _duration_to_sec(points[-1].time_from_start)
        period = 1.0 / self.rate_hz
        next_tick = start_time
        last_positions = {side: list(starts[side]) for side in involved}
        velocity_available = all(
            timeline[3] for timeline in timelines.values())
        self.get_logger().info(
            f'Executing {len(points)} trajectory points in {end_time:.3f}s; '
            f'interpolation={self.interpolation}, '
            f'trajectory_velocities={velocity_available}')
        command_count = 0
        max_command_gap = 0.0
        last_command_time = None

        try:
            while rclpy.ok():
                with self._cancel_lock:
                    cancel_requested = self._cancel_requested
                if cancel_requested:
                    self.get_logger().warn(
                        f'Trajectory cancelled by cancel service; dry_run={self.dry_run}')
                    self._emergency_stop(involved)
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    return result
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn(
                        f'Trajectory cancelled; dry_run={self.dry_run}')
                    self._emergency_stop(involved)
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    return result

                elapsed = time.monotonic() - start_time
                t = min(elapsed, end_time)
                side_targets = {
                    side: _interpolate_side(
                        timelines[side], t, self.interpolation == 'cubic')
                    for side in involved
                }

                for side in involved:
                    last_positions[side] = side_targets[side]
                    if self.dry_run:
                        self.sim_positions[side] = list(side_targets[side])
                    else:
                        self._publish_command(side, side_targets[side])
                command_time = time.monotonic()
                if last_command_time is not None:
                    max_command_gap = max(
                        max_command_gap, command_time - last_command_time)
                last_command_time = command_time
                command_count += 1

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
                next_tick += period
                now = time.monotonic()
                if now < next_tick:
                    time.sleep(next_tick - now)
                elif now - next_tick > period:
                    next_tick = now

            hold_start = time.monotonic()
            hold_deadline = hold_start + self.final_hold_sec
            settle_deadline = hold_deadline
            if not self.dry_run:
                settle_deadline += self.settle_timeout_sec
            next_hold_tick = hold_start
            settled = self.dry_run
            final_error_vectors = {}
            final_signed_vectors = {}
            settle_samples = []
            last_settle_sample = None
            settle_sample_interval = 0.25
            if self.final_hold_sec > 0.0 or self.settle_timeout_sec > 0.0:
                while rclpy.ok() and time.monotonic() < settle_deadline:
                    for side in involved:
                        if self.dry_run:
                            self.sim_positions[side] = list(last_positions[side])
                        else:
                            self._publish_command(side, last_positions[side])
                    now = time.monotonic()
                    if not self.dry_run:
                        final_signed_vectors = self._signed_tracking_errors(
                            involved, last_positions)
                        if now >= hold_deadline:
                            final_error_vectors = {
                                side: [abs(error) for error in errors]
                                for side, errors in final_signed_vectors.items()
                            }
                            if last_settle_sample is None or (
                                    now - last_settle_sample
                                    >= settle_sample_interval - 1e-6):
                                settle_samples.append((
                                    now - hold_deadline,
                                    {
                                        side: list(errors)
                                        for side, errors in final_signed_vectors.items()
                                    },
                                ))
                                last_settle_sample = now
                            if final_error_vectors and max(
                                    max(errors)
                                    for errors in final_error_vectors.values()
                            ) <= self.settle_tolerance_rad:
                                settled = True
                                break
                    next_hold_tick += period
                    if now < next_hold_tick:
                        time.sleep(next_hold_tick - now)
                    elif now - next_hold_tick > period:
                        next_hold_tick = now

            if not self.dry_run and not final_signed_vectors:
                final_signed_vectors = self._signed_tracking_errors(
                    involved, last_positions)
            if not final_error_vectors:
                final_error_vectors = {
                    side: [abs(error) for error in errors]
                    for side, errors in final_signed_vectors.items()
                }
            if final_error_vectors and max(
                    max(errors) for errors in final_error_vectors.values()
            ) <= self.settle_tolerance_rad:
                settled = True
            final_errors = {
                side: max(errors)
                for side, errors in final_error_vectors.items()
            }
            worst_joints = {
                side: errors.index(max(errors)) + 1
                for side, errors in final_error_vectors.items()
            }
            signed_final_errors = {
                side: final_signed_vectors[side][errors.index(max(errors))]
                for side, errors in final_error_vectors.items()
            }
            worst_targets = {
                side: last_positions[side][errors.index(max(errors))]
                for side, errors in final_error_vectors.items()
            }
            worst_actuals = {
                side: self.feedback[side][errors.index(max(errors))]
                for side, errors in final_error_vectors.items()
                if self.feedback[side] is not None
            }
            settle_traces = {
                side: [
                    (round(elapsed, 3), round(errors[side][worst_index], 6))
                    for elapsed, errors in settle_samples
                    if side in errors
                ]
                for side, worst_index in (
                    (side, errors.index(max(errors)))
                    for side, errors in final_error_vectors.items()
                )
            }
            if not self.dry_run and not settled:
                self.get_logger().warn(
                    f'Final tracking error did not settle within '
                    f'{self.settle_timeout_sec:.3f}s; errors={final_errors}, '
                    f'signed_errors={signed_final_errors}, '
                    f'worst_joints={worst_joints}, '
                    f'worst_target_rad={worst_targets}, '
                    f'worst_actual_rad={worst_actuals}, '
                    f'settle_trace={settle_traces}')
            if not self.dry_run:
                for side in involved:
                    self._call_setbool(side, False)
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            goal_handle.succeed()
            self.get_logger().info(
                f'Trajectory completed in {end_time:.3f}s; dry_run={self.dry_run}, '
                f'commands={command_count}, '
                f'max_command_gap={max_command_gap * 1000.0:.1f}ms, '
                f'final_error_rad={final_errors}, '
                f'signed_error_rad={signed_final_errors}, '
                f'worst_joints={worst_joints}, '
                f'worst_target_rad={worst_targets}, '
                f'worst_actual_rad={worst_actuals}, '
                f'settle_trace={settle_traces}, settled={settled}')
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
