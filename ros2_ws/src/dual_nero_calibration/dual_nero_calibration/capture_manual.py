#!/usr/bin/env python3
"""Manually capture one calibration sample at a time."""
from __future__ import annotations

import argparse
from pathlib import Path
import select
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .board_quality import detect_chessboard
from .frame_selection import nearest_pose, pose_rotation_spread_deg
from .session import make_session_dir, save_sample, write_metadata
from .common import (
    default_deployment_dir,
    invert_transform,
    load_yaml,
    transform_from_msg,
)


class ManualCapture(Node):
    def __init__(self, args) -> None:
        super().__init__('capture_manual')
        self.args = args
        share = Path(get_package_share_directory('dual_nero_calibration'))
        targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
        if args.camera not in targets:
            raise RuntimeError(f'camera must be one of {sorted(targets)}')
        self.camera_cfg = targets[args.camera]
        self.board_cfg = load_yaml(
            share / 'config' / 'calibration_board.yaml')['board']
        if self.board_cfg.get('type', 'chessboard') != 'chessboard':
            raise RuntimeError('manual capture currently supports chessboard only')

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
        self.session_dir = make_session_dir(root, args.camera, 'manual')

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_image = None
        self.latest_image_time = 0.0
        self.latest_info = None
        self.saved: list[dict] = []
        self.lock = threading.Lock()
        self.finished = False
        self.last_status_time = 0.0
        self.last_status_text = ''

        self.create_subscription(
            CameraInfo, self.camera_cfg['info_topic'], self._info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.camera_cfg['image_topic'], self._image_cb,
            qos_profile_sensor_data)
        self.create_service(Trigger, '~/capture', self._capture_service)
        self.create_service(Trigger, '~/finish', self._finish_service)

        self.get_logger().info(
            f'Manual capture for {args.camera} into {self.session_dir}')
        self.get_logger().info(
            'Press Enter to capture, s=status, d=delete last, q=finish. '
            'Service: /capture_manual/capture')

    def _info_cb(self, msg: CameraInfo) -> None:
        self.latest_info = msg

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg
        self.latest_image_time = time.monotonic()

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

    def _tool_motion(self, stamp_ns: int):
        if self.camera_cfg['mode'] != 'eye_in_hand':
            return 0.0, 0.0, None
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
            previous = previous_result[0]
            delta = invert_transform(previous) @ current
            max_translation = max(
                max_translation, float(np.linalg.norm(delta[:3, 3])))
            max_rotation = max(
                max_rotation, float(np.degrees(np.linalg.norm(
                    Rotation.from_matrix(delta[:3, :3]).as_rotvec()))))
        return max_translation, max_rotation, current

    def _detect_latest(self):
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
        return image_msg, info, K, D, detection

    def _capture(self) -> tuple[bool, str]:
        with self.lock:
            try:
                image_msg, info, K, D, detection = self._detect_latest()
            except Exception as exc:  # noqa: BLE001
                return False, str(exc)

            stamp_ns = self._stamp_ns(image_msg)
            motion = self._tool_motion(stamp_ns)
            if motion is None:
                return False, 'robot TF unavailable at image time'
            max_translation, max_rotation, T_base_tool = motion
            if self.camera_cfg['mode'] != 'eye_in_hand':
                T_base_tool = np.eye(4)
            if (max_translation > self.args.max_stationary_translation_mm / 1000.0
                    or max_rotation > self.args.max_stationary_rotation_deg):
                return False, (
                    f'robot not stationary: last 0.3 s moved '
                    f'{max_translation * 1000:.2f} mm / {max_rotation:.3f} deg; '
                    'wait a moment and retry')

            T_link_optical = self._lookup(
                self.camera_cfg['camera_link_frame'],
                self.camera_cfg['optical_frame'], stamp_ns)
            if T_link_optical is None:
                return False, 'camera link/optical TF unavailable'

            if self.camera_cfg['mode'] == 'eye_in_hand':
                similar = self._nearest_existing_pose(T_base_tool)
                if similar is not None:
                    index, translation, rotation = similar
                    return False, (
                        f'almost the same pose as sample {index}: '
                        f'{translation * 1000:.1f} mm / {rotation:.2f} deg. '
                        'Move further before recording; pass '
                        '--allow-near-duplicate to keep it anyway')

            path = save_sample(
                self.session_dir, len(self.saved),
                camera_matrix=K,
                distortion=D,
                image_size=(image_msg.width, image_msg.height),
                object_points=detection.object_points,
                image_points=detection.image_points,
                T_cam_board=detection.T_cam_board,
                T_base_tool=T_base_tool,
                T_camera_link_optical=T_link_optical,
                reprojection_rms_px=detection.reprojection_rms_px,
                stamp_sec=int(image_msg.header.stamp.sec),
                stamp_nanosec=int(image_msg.header.stamp.nanosec))
            self.saved.append({
                'file': path.name,
                'T_base_tool': T_base_tool.tolist(),
                'reprojection_rms_px': float(detection.reprojection_rms_px),
                'min_corner_gap_px': float(detection.min_corner_gap_px),
                'stationary_translation_mm': float(max_translation * 1000.0),
                'stationary_rotation_deg': float(max_rotation),
            })
            self._write_metadata()
            message = (
                f'captured {len(self.saved)}: {path.name} '
                f'rms={detection.reprojection_rms_px:.3f} px '
                f'motion={max_translation * 1000:.2f} mm/'
                f'{max_rotation:.3f} deg')
            if self.args.samples > 0 and len(self.saved) >= self.args.samples:
                self.finished = True
                message += ' (target reached)'
            return True, message

    def _write_metadata(self) -> None:
        write_metadata(
            self.session_dir, camera=self.args.camera,
            camera_cfg=self.camera_cfg, board_cfg=self.board_cfg,
            samples=self.saved, capture_mode='manual')

    def delete_last(self) -> tuple[bool, str]:
        with self.lock:
            if not self.saved:
                return False, 'no sample to delete'
            entry = self.saved.pop()
            path = self.session_dir / entry['file']
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            self._write_metadata()
            return True, f"deleted {entry['file']}; {len(self.saved)} remaining"

    def _nearest_existing_pose(self, T_base_tool):
        """Return the closest already-recorded pose if it is too similar."""
        if self.args.allow_near_duplicate or not self.saved:
            return None
        return nearest_pose(
            T_base_tool,
            [entry.get('T_base_tool') for entry in self.saved],
            self.args.min_pose_rotation_deg,
            self.args.min_pose_translation_mm)

    def rotation_spread_deg(self) -> float:
        return pose_rotation_spread_deg(
            [entry['T_base_tool'] for entry in self.saved
             if 'T_base_tool' in entry])

    def _capture_service(self, request, response):
        ok, message = self._capture()
        response.success = ok
        response.message = message
        return response

    def _finish_service(self, request, response):
        self.finished = True
        response.success = True
        response.message = f'finish requested after {len(self.saved)} samples'
        return response

    def status(self) -> str:
        try:
            image_msg, _, _, _, detection = self._detect_latest()
        except Exception as exc:  # noqa: BLE001
            return f'board=NO ({exc})'
        motion = self._tool_motion(self._stamp_ns(image_msg))
        if motion is None:
            return f'board=OK rms={detection.reprojection_rms_px:.3f}px ' \
                   f'tf=NO'
        translation, rotation, _ = motion
        stationary = (
            translation <= self.args.max_stationary_translation_mm / 1000.0
            and rotation <= self.args.max_stationary_rotation_deg)
        return (
            f'board=OK rms={detection.reprojection_rms_px:.3f}px '
            f'motion={translation * 1000:.2f}mm/{rotation:.3f}deg '
            f'stationary={"YES" if stationary else "NO"} '
            f'spread={self.rotation_spread_deg():.1f}deg')

    def print_status(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_status_time < self.args.status_interval_sec:
            return
        self.last_status_time = now
        text = self.status()
        if text != self.last_status_text or force:
            print(f'[{len(self.saved)} saved] {text}', flush=True)
            self.last_status_text = text


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', required=True,
                        choices=('scene', 'left_wrist', 'right_wrist'))
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--samples', type=int, default=0,
                        help='Stop automatically after N samples; 0 = unlimited')
    parser.add_argument('--max-reprojection-rms-px', type=float, default=1.0)
    parser.add_argument('--min-corner-gap-px', type=float, default=2.0)
    parser.add_argument('--max-stationary-translation-mm', type=float,
                        default=1.0)
    parser.add_argument('--max-stationary-rotation-deg', type=float, default=0.2)
    parser.add_argument('--max-image-age-sec', type=float, default=2.0)
    parser.add_argument('--status-interval-sec', type=float, default=3.0)
    parser.add_argument('--min-pose-rotation-deg', type=float, default=3.0,
                        help='Reject a frame this close in rotation to an '
                             'existing sample')
    parser.add_argument('--min-pose-translation-mm', type=float, default=10.0,
                        help='Reject a frame this close in translation to an '
                             'existing sample')
    parser.add_argument('--allow-near-duplicate', action='store_true',
                        help='Keep near-duplicate poses anyway')
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init(args=[])
    node = ManualCapture(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin() -> None:
        try:
            executor.spin()
        except Exception:  # noqa: BLE001 - shutdown races are expected
            pass

    spin_thread = threading.Thread(target=spin, daemon=True)
    spin_thread.start()
    print('Enter=capture  s=status  d=delete last  q=finish', flush=True)
    try:
        while rclpy.ok() and not node.finished:
            if sys.stdin.isatty():
                ready, _, _ = select.select([sys.stdin], [], [], 0.2)
                if ready:
                    raw = sys.stdin.readline()
                    if raw == '':
                        break
                    line = raw.strip().lower()
                    if line in ('q', 'quit', 'exit'):
                        break
                    if line in ('d', 'delete', 'u', 'undo'):
                        print(node.delete_last()[1], flush=True)
                    elif line in ('s', 'status'):
                        node.print_status(force=True)
                    else:
                        ok, message = node._capture()
                        print(message, flush=True)
                        if ok:
                            node.print_status(force=True)
            else:
                time.sleep(0.2)
                node.print_status()
    except KeyboardInterrupt:
        pass
    finally:
        node._write_metadata()
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f'finished: {len(node.saved)} samples in {node.session_dir}',
              flush=True)


if __name__ == '__main__':
    main()
