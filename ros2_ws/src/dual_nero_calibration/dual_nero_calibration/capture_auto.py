#!/usr/bin/env python3
"""Drive the arm through planned poses and capture chessboard samples.

Motion goes through the existing ``/dual_arm_controller/follow_joint_trajectory``
action, so the MoveIt trajectory bridge still owns gating, limits and settling.
Planning is joint-space: poses are sampled around the pose where the board was
first seen, and only those that keep the board in view are kept.

Without ``--execute`` nothing moves; the planned poses are printed only.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from cv_bridge import CvBridge
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint

from .board_quality import detect_chessboard
from .common import (
    default_deployment_dir,
    invert_transform,
    load_yaml,
    transform_from_msg,
)
from .kinematics import load_arm_chain
from .pose_planner import (
    board_corners_in_base,
    generate_candidates,
    view_ok,
)
from .session import make_session_dir, save_sample, write_metadata


def _plausible_mount(pose: np.ndarray, max_offset_m: float = 0.5) -> bool:
    """A wrist camera sits on the tool, not metres away from it."""
    return bool(np.linalg.norm(pose[:3, 3]) <= max_offset_m)


def _as_list(joints) -> list[float]:
    return [] if joints is None else [float(v) for v in joints]


class AutoCapture(Node):
    def __init__(self, args) -> None:
        super().__init__('capture_auto')
        self.args = args
        share = Path(get_package_share_directory('dual_nero_calibration'))
        targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
        if args.camera not in targets:
            raise RuntimeError(f'camera must be one of {sorted(targets)}')
        self.camera_cfg = targets[args.camera]
        if self.camera_cfg['mode'] != 'eye_in_hand':
            raise RuntimeError('auto capture only supports eye-in-hand cameras')
        self.board_cfg = load_yaml(
            share / 'config' / 'calibration_board.yaml')['board']
        if self.board_cfg.get('type', 'chessboard') != 'chessboard':
            raise RuntimeError('auto capture supports chessboard only')

        self.side = args.camera.split('_')[0]
        self.chain = load_arm_chain(self.side, xacro_args=args.xacro_args)

        self.pattern_size = (
            int(self.board_cfg['pattern_cols']),
            int(self.board_cfg['pattern_rows']),
        )
        square = float(self.board_cfg['square_length_m'])
        grid = np.mgrid[
            0:self.pattern_size[0], 0:self.pattern_size[1]
        ].T.reshape(-1, 2).astype(np.float32)
        self.object_points = np.column_stack(
            [grid * square, np.zeros(len(grid), dtype=np.float32)])

        root = (
            Path(args.output_dir).expanduser()
            if args.output_dir else default_deployment_dir('calibration_data'))
        self.session_dir = make_session_dir(root, args.camera, 'auto')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_image = None
        self.latest_image_time = 0.0
        self.latest_info = None
        self.joints: dict[str, float] = {}
        self.saved: list[dict] = []
        self.active_goal = None
        self.seed_joints = None
        self.lock = threading.Lock()
        self.action = ActionClient(
            self, FollowJointTrajectory,
            args.trajectory_action)

        self.create_subscription(
            CameraInfo, self.camera_cfg['info_topic'], self._info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.camera_cfg['image_topic'], self._image_cb,
            qos_profile_sensor_data)
        for topic in args.joint_state_topics:
            self.create_subscription(JointState, topic, self._joint_cb,
                                     qos_profile_sensor_data)

    def _info_cb(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_time = time.monotonic()

    def _joint_cb(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self.joints[name] = float(position)

    def current_joints(self):
        values = [self.joints.get(name) for name in self.chain.joint_names]
        if any(value is None for value in values):
            return None
        return np.array(values, dtype=np.float64)

    def wait_ready(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (self.current_joints() is not None
                    and self.latest_image is not None
                    and self.latest_info is not None):
                return
            time.sleep(0.1)
        raise RuntimeError('timed out waiting for joints, image and camera info')

    def _stamp_ns(self, msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec)

    def _lookup_pose(self, target: str, source: str, stamp_ns: int):
        candidates = [int(stamp_ns), int(stamp_ns - 20_000_000),
                      int(stamp_ns - 50_000_000), 0]
        for candidate in candidates:
            try:
                msg = self.tf_buffer.lookup_transform(
                    target, source, Time(nanoseconds=candidate))
            except Exception:
                continue
            actual_ns = (
                int(msg.header.stamp.sec) * 1_000_000_000 +
                int(msg.header.stamp.nanosec))
            if actual_ns == 0:
                actual_ns = candidate or stamp_ns
            return transform_from_msg(msg.transform), actual_ns
        return None

    def _lookup(self, target: str, source: str, stamp_ns: int):
        result = self._lookup_pose(target, source, stamp_ns)
        return None if result is None else result[0]

    def _planning_extrinsic(self):
        """Nominal tool0 -> camera_link, used only to predict the view."""
        pose = self._lookup(self.camera_cfg['tool_frame'],
                            self.camera_cfg['camera_link_frame'], 0)
        if pose is not None and _plausible_mount(pose):
            self.get_logger().info('planner uses tool0 -> camera_link from TF')
            return pose
        if pose is not None:
            self.get_logger().warn(
                'published tool0 -> camera_link looks implausible '
                f'({np.linalg.norm(pose[:3, 3]) * 100:.1f} cm from the tool); '
                'ignoring it for planning')
        results = default_deployment_dir('calibration_results')
        candidates = []
        if self.args.extrinsic:
            candidates.append(Path(self.args.extrinsic))
        # Newest result for this camera; a WARN file is still good enough to
        # predict the field of view, which is all the planner uses it for.
        found = sorted(
            results.glob(f'{self.args.camera}_extrinsic*.yaml'),
            key=lambda path: path.stat().st_mtime, reverse=True)
        candidates.extend(found)
        for path in candidates:
            if not path.is_file():
                continue
            data = load_yaml(path)['extrinsic']
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(
                data['quaternion_xyzw']).as_matrix()
            pose[:3, 3] = np.asarray(data['translation'], dtype=np.float64)
            if not _plausible_mount(pose):
                continue
            if data.get('status', 'PASS') != 'PASS':
                self.get_logger().warn(
                    f'planner uses {path.name} (status '
                    f'{data.get("status")}); fine for view prediction, do not '
                    'publish it')
            else:
                self.get_logger().info(
                    f'planner uses tool0 -> camera_link from {path}')
            return pose
        return None

    def _tool_motion(self, stamp_ns: int):
        current_result = self._lookup_pose(
            self.camera_cfg['base_frame'], self.camera_cfg['tool_frame'],
            stamp_ns)
        if current_result is None:
            return None
        current, current_stamp = current_result
        max_translation = 0.0
        max_rotation = 0.0
        for offset_sec in (0.1, 0.2, 0.3):
            previous_result = self._lookup_pose(
                self.camera_cfg['base_frame'], self.camera_cfg['tool_frame'],
                current_stamp - int(offset_sec * 1_000_000_000))
            if previous_result is None:
                return None
            delta = invert_transform(previous_result[0]) @ current
            max_translation = max(
                max_translation, float(np.linalg.norm(delta[:3, 3])))
            max_rotation = max(max_rotation, float(np.degrees(
                np.linalg.norm(Rotation.from_matrix(delta[:3, :3]).as_rotvec()))))
        return max_translation, max_rotation, current

    def detect_latest(self):
        if self.latest_image is None:
            raise RuntimeError('no image received yet')
        if self.latest_info is None:
            raise RuntimeError('no CameraInfo received yet')
        if time.monotonic() - self.latest_image_time > self.args.max_image_age_sec:
            raise RuntimeError('latest image is stale')
        image_msg = self.latest_image
        info = self.latest_info
        try:
            image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(f'image conversion failed: {exc}') from exc
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(info.d, dtype=np.float64)
        detection = detect_chessboard(
            gray, self.pattern_size, self.object_points, K, D,
            min_corner_gap_px=self.args.min_corner_gap_px,
            max_reprojection_rms_px=self.args.max_reprojection_rms_px)
        return image_msg, image, K, D, detection

    def _wait_future(self, future, timeout: float):
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.02)
        return future.done()

    def capture(self) -> tuple[bool, str]:
        with self.lock:
            try:
                image_msg, image, K, D, detection = self.detect_latest()
            except Exception as exc:  # noqa: BLE001
                return False, str(exc)
            stamp_ns = self._stamp_ns(image_msg)
            motion = self._tool_motion(stamp_ns)
            if motion is None:
                return False, 'robot TF unavailable at image time'
            max_translation, max_rotation, T_base_tool = motion
            if (max_translation > self.args.max_stationary_translation_mm / 1000.0
                    or max_rotation > self.args.max_stationary_rotation_deg):
                return False, (
                    f'robot not stationary: last 0.3 s moved '
                    f'{max_translation * 1000:.2f} mm / {max_rotation:.3f} deg')
            T_link_optical = self._lookup(
                self.camera_cfg['camera_link_frame'],
                self.camera_cfg['optical_frame'], stamp_ns)
            if T_link_optical is None:
                return False, 'camera link/optical TF unavailable'

            index = len(self.saved)
            path = save_sample(
                self.session_dir, index,
                camera_matrix=K, distortion=D,
                image_size=(image_msg.width, image_msg.height),
                object_points=detection.object_points,
                image_points=detection.image_points,
                T_cam_board=detection.T_cam_board,
                T_base_tool=T_base_tool,
                T_camera_link_optical=T_link_optical,
                reprojection_rms_px=detection.reprojection_rms_px,
                stamp_sec=int(image_msg.header.stamp.sec),
                stamp_nanosec=int(image_msg.header.stamp.nanosec))
            entry = {
                'file': path.name,
                'joints': _as_list(self.current_joints()),
                'reprojection_rms_px': float(detection.reprojection_rms_px),
                'min_corner_gap_px': float(detection.min_corner_gap_px),
                'stationary_translation_mm': float(max_translation * 1000.0),
                'stationary_rotation_deg': float(max_rotation),
            }
            if self.args.save_images:
                image_path = self.session_dir / f'sample_{index:04d}.png'
                cv2.imwrite(str(image_path), image)
                entry['image'] = image_path.name
                overlay = image.copy()
                cv2.drawChessboardCorners(
                    overlay, self.pattern_size,
                    detection.image_points.reshape(-1, 1, 2).astype(np.float32),
                    True)
                overlay_path = self.session_dir / f'sample_{index:04d}_detected.png'
                cv2.imwrite(str(overlay_path), overlay)
                entry['image_detected'] = overlay_path.name
            self.saved.append(entry)
            write_metadata(self.session_dir, camera=self.args.camera,
                           camera_cfg=self.camera_cfg,
                           board_cfg=self.board_cfg, samples=self.saved,
                           capture_mode='auto')
            return True, (
                f'captured {index + 1}/{self.args.samples}: {path.name} '
                f'rms={detection.reprojection_rms_px:.3f} px')

    def wait_stationary(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and rclpy.ok():
            if self.latest_image is None:
                time.sleep(0.05)
                continue
            motion = self._tool_motion(self._stamp_ns(self.latest_image))
            if motion is None:
                time.sleep(0.05)
                continue
            translation, rotation, _ = motion
            if (translation <= self.args.max_stationary_translation_mm / 1000.0
                    and rotation <= self.args.max_stationary_rotation_deg):
                return True
            time.sleep(0.05)
        return False

    def move_to(self, target: np.ndarray) -> bool:
        start = self.current_joints()
        if start is None:
            raise RuntimeError('joint state unavailable before motion')
        delta = float(np.max(np.abs(target - start)))
        duration = max(self.args.min_move_sec,
                       delta / max(self.args.joint_speed_rad_s, 1e-3))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.chain.joint_names
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in target]
        point.velocities = [0.0] * len(target)
        point.time_from_start = RosDuration(seconds=duration).to_msg()
        goal.trajectory.points = [point]
        if not self.action.wait_for_server(timeout_sec=5.0):
            raise RuntimeError(
                f'trajectory action {self.args.trajectory_action} unavailable')
        future = self.action.send_goal_async(goal)
        if not self._wait_future(future, 10.0):
            self.get_logger().error('trajectory goal was not accepted in time')
            return False
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('trajectory goal rejected')
            return False
        self.active_goal = handle
        result_future = handle.get_result_async()
        if not self._wait_future(result_future, duration + 20.0):
            self.get_logger().error('trajectory result timed out')
            handle.cancel_goal_async()
            return False
        self.active_goal = None
        result = result_future.result()
        code = result.result.error_code
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().error(f'trajectory failed with error_code={code}')
            return False
        if not self.wait_stationary(self.args.stationary_timeout_sec):
            self.get_logger().error('arm did not settle after the trajectory')
            return False
        # A dry-run bridge reports success without moving; that would silently
        # produce a session of identical poses.
        reached = self.current_joints()
        error = (float(np.max(np.abs(reached - target)))
                 if reached is not None else float('inf'))
        if error > self.args.reach_tolerance_rad:
            self.get_logger().error(
                f'arm stopped {np.degrees(error):.1f} deg away from the '
                'target joint position; is the trajectory bridge in dry_run?')
            return False
        return True

    def check_fk(self) -> tuple[bool, str]:
        joints = self.current_joints()
        if joints is None:
            return False, 'joint state unavailable'
        fk_pose = self.chain.forward(joints)
        tf_pose = self._lookup(self.camera_cfg['base_frame'],
                               self.camera_cfg['tool_frame'], 0)
        if tf_pose is None:
            return False, 'world -> tool0 TF unavailable'
        delta = invert_transform(tf_pose) @ fk_pose
        distance = float(np.linalg.norm(delta[:3, 3])) * 1000.0
        angle = float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
        if distance > self.args.fk_tolerance_mm or angle > self.args.fk_tolerance_deg:
            return False, (
                f'URDF FK disagrees with TF by {distance:.1f} mm / '
                f'{angle:.2f} deg; launch the description with the same '
                'xacro arguments or pass --xacro-args')
        return True, f'FK matches TF within {distance:.2f} mm / {angle:.2f} deg'

    def plan(self, count: int):
        ok, message = self.check_fk()
        self.get_logger().info(f'FK self-check: {message}')
        if not ok:
            raise RuntimeError(message)
        self.seed_joints = self.current_joints()

        captured, message = self.capture()
        if not captured:
            raise RuntimeError(f'seed capture failed: {message}')
        seed = self.saved[-1]
        seed_sample = np.load(self.session_dir / seed['file'])
        T_base_tool = seed_sample['T_base_tool']
        T_cam_board = seed_sample['T_cam_board']
        T_link_optical = seed_sample['T_camera_link_optical']
        T_tool_camera_link = self._planning_extrinsic()
        if T_tool_camera_link is None:
            raise RuntimeError(
                'no tool0 -> camera TF and no --extrinsic YAML; the planner '
                'needs a nominal hand-eye transform (it does not have to be '
                'calibrated, just close enough to predict the view)')
        T_tool_camera = T_tool_camera_link @ T_link_optical
        T_base_board = T_base_tool @ T_tool_camera @ T_cam_board
        self.get_logger().info(
            'seed captured; board origin in base frame: '
            f'{np.round(T_base_board[:3, 3], 3).tolist()}')

        K = seed_sample['camera_matrix']
        image_size = seed_sample['image_size']
        candidates = generate_candidates(
            chain=self.chain,
            q_seed=self.current_joints(),
            T_tool_camera=T_tool_camera,
            T_base_board=T_base_board,
            object_points=self.object_points,
            camera_matrix=K, image_size=image_size,
            pattern_size=self.pattern_size,
            count=count, seed=self.args.seed,
            max_step_deg=self.args.max_joint_step_deg,
            limit_margin_deg=self.args.limit_margin_deg)
        corners = board_corners_in_base(T_base_board, self.object_points)
        aborted = False
        for index, candidate in enumerate(candidates):
            ok, reason, score = view_ok(
                candidate.T_base_tool @ T_tool_camera, corners, K,
                image_size, self.pattern_size)
            delta = candidate.joints - self.current_joints()
            self.get_logger().info(
                f'pose {index:02d}: joint delta '
                f'{np.round(np.degrees(delta), 1).tolist()} '
                f'view={"ok" if ok else reason} score={score:.2f}')
        return candidates

    def run(self) -> int:
        self.wait_ready(self.args.startup_timeout_sec)
        plan_count = max(0, self.args.samples - 1)
        if plan_count <= 0:
            self._finish()
            return 0
        candidates = self.plan(plan_count)
        if not candidates:
            self.get_logger().error(
                'no pose kept the board in view; move the arm closer to the '
                'board or increase --max-joint-step-deg')
            self._finish()
            return 2
        if not self.args.execute:
            self.get_logger().warn(
                f'planned {len(candidates)} poses; nothing was moved. '
                'Re-run with --execute to move the arm.')
            self._finish()
            return 0

        for index, candidate in enumerate(candidates):
            if len(self.saved) >= self.args.samples:
                break
            self.get_logger().info(
                f'moving to pose {index + 1}/{len(candidates)}')
            if not self.move_to(candidate.joints):
                self.get_logger().error(
                    f'pose {index + 1} failed to execute; stopping')
                aborted = True
                break
            ok, message = self.capture()
            self.get_logger().info(message)
            if ok:
                continue
            self.get_logger().warn(
                f'capture rejected at pose {index + 1}; continuing')
        self._finish()
        if (not aborted and self.args.return_to_seed
                and self.seed_joints is not None):
            self.get_logger().info('returning to the seed pose')
            self.move_to(self.seed_joints)
        return 0 if len(self.saved) >= self.args.samples else 1

    def _finish(self) -> None:
        write_metadata(self.session_dir, camera=self.args.camera,
                       camera_cfg=self.camera_cfg, board_cfg=self.board_cfg,
                       samples=self.saved, capture_mode='auto')
        self.get_logger().info(
            f'{len(self.saved)} samples in {self.session_dir}')


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', required=True,
                        choices=('left_wrist', 'right_wrist'))
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--execute', action='store_true',
                        help='Actually move the arm; without it only plans')
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--trajectory-action', default=(
        '/dual_arm_controller/follow_joint_trajectory'))
    parser.add_argument('--joint-state-topics', nargs='+', default=[
        '/joint_states', '/left/feedback/joint_states',
        '/right/feedback/joint_states'])
    parser.add_argument('--xacro-args', nargs='+', default=[],
                        help='key:=value overrides for the description URDF')
    parser.add_argument('--save-images', dest='save_images',
                        action='store_true', default=True,
                        help='Save the raw and detected PNG for each sample')
    parser.add_argument('--no-save-images', dest='save_images',
                        action='store_false')
    parser.add_argument('--extrinsic', default='',
                        help='Fallback hand-eye YAML used only for planning')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-joint-step-deg', type=float, default=22.0)
    parser.add_argument('--limit-margin-deg', type=float, default=4.0)
    parser.add_argument('--joint-speed-rad-s', type=float, default=0.35)
    parser.add_argument('--min-move-sec', type=float, default=2.0)
    parser.add_argument('--stationary-timeout-sec', type=float, default=8.0)
    parser.add_argument('--startup-timeout-sec', type=float, default=20.0)
    parser.add_argument('--max-stationary-translation-mm', type=float,
                        default=1.0)
    parser.add_argument('--max-stationary-rotation-deg', type=float, default=0.2)
    parser.add_argument('--max-image-age-sec', type=float, default=2.0)
    parser.add_argument('--max-reprojection-rms-px', type=float, default=1.0)
    parser.add_argument('--min-corner-gap-px', type=float, default=2.0)
    parser.add_argument('--fk-tolerance-mm', type=float, default=10.0)
    parser.add_argument('--fk-tolerance-deg', type=float, default=1.0)
    parser.add_argument('--reach-tolerance-rad', type=float, default=0.05,
                        help='Reject a move that did not reach the target '
                             '(catches a dry-run trajectory bridge)')
    parser.add_argument('--return-to-seed', action='store_true', default=True)
    parser.add_argument('--no-return-to-seed', dest='return_to_seed',
                        action='store_false')
    args = parser.parse_args()
    args.xacro_args = _parse_xacro_args(args.xacro_args)
    return args


def _parse_xacro_args(values) -> dict:
    result = {}
    for value in values:
        if ':=' not in value:
            raise SystemExit(f'--xacro-args expects key:=value, got {value!r}')
        key, raw = value.split(':=', 1)
        result[key.strip()] = raw.strip()
    return result


def main() -> int:
    args = parse_args()
    rclpy.init(args=[])
    node = AutoCapture(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        return node.run()
    except KeyboardInterrupt:
        if node.active_goal is not None:
            node.active_goal.cancel_goal_async()
        node.get_logger().warn('interrupted; cancelling the active motion goal')
        return 130
    finally:
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    import sys
    sys.exit(main())
