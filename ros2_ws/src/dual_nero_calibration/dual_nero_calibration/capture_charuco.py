#!/usr/bin/env python3
"""Capture ChArUco calibration samples from one RealSense camera."""
from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener

from .common import invert_transform, load_yaml, transform_from_msg


class CharucoCapture(Node):
    def __init__(self) -> None:
        super().__init__('capture_charuco')

        self.declare_parameter('camera', 'left_wrist')
        self.declare_parameter('samples', 20)
        self.declare_parameter('output_dir', '/home/a/workspace/dual_nero_deployment/calibration_data')
        self.declare_parameter('min_interval_sec', 1.0)
        self.declare_parameter('min_tool_motion_m', 0.02)
        self.declare_parameter('min_tool_rotation_deg', 2.0)
        self.declare_parameter('max_seconds', 180.0)

        self.camera = self.get_parameter('camera').value
        self.requested_samples = int(self.get_parameter('samples').value)
        self.output_root = Path(self.get_parameter('output_dir').value)
        self.min_interval = float(self.get_parameter('min_interval_sec').value)
        self.min_tool_motion = float(self.get_parameter('min_tool_motion_m').value)
        self.min_tool_rotation = float(self.get_parameter('min_tool_rotation_deg').value)
        self.max_seconds = float(self.get_parameter('max_seconds').value)

        share = Path(get_package_share_directory('dual_nero_calibration'))
        targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
        if self.camera not in targets:
            raise RuntimeError(f'camera must be one of {sorted(targets)}')
        self.config = targets[self.camera]
        board_cfg = load_yaml(share / 'config' / 'calibration_board.yaml')['board']

        self.board_cfg = board_cfg
        self.board_type = board_cfg.get('type', 'chessboard')
        self.dictionary = None
        self.board = None
        self.chessboard_object_points = None
        if self.board_type == 'chessboard':
            self.pattern_size = (
                int(board_cfg['pattern_cols']),
                int(board_cfg['pattern_rows']),
            )
            square = float(board_cfg['square_length_m'])
            grid = np.mgrid[
                0:self.pattern_size[0], 0:self.pattern_size[1]
            ].T.reshape(-1, 2).astype(np.float32)
            self.chessboard_object_points = np.column_stack(
                [grid * square, np.zeros(len(grid), dtype=np.float32)])
            self.get_logger().info(
                f'Using chessboard inner corners {self.pattern_size}, '
                f'square={square:.4f} m')
        elif self.board_type == 'charuco':
            dictionary_name = board_cfg['dictionary']
            if not hasattr(cv2.aruco, dictionary_name):
                raise RuntimeError(f'Unknown ArUco dictionary: {dictionary_name}')
            self.dictionary = cv2.aruco.Dictionary_get(
                getattr(cv2.aruco, dictionary_name))
            self.board = cv2.aruco.CharucoBoard_create(
                int(board_cfg['squares_x']),
                int(board_cfg['squares_y']),
                float(board_cfg['square_length_m']),
                float(board_cfg['marker_length_m']),
                self.dictionary,
            )
            self.min_corners = int(board_cfg['min_charuco_corners'])
        else:
            raise RuntimeError(f'Unsupported board type: {self.board_type}')

        session_name = time.strftime('%Y%m%d_%H%M%S')
        self.session_dir = self.output_root / self.camera / session_name
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self.camera_info = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_capture = 0.0
        self.last_tool_pose = None
        self.sample_count = 0
        self.started = time.monotonic()
        self.done = False
        self.saved_meta = []

        self.create_subscription(
            CameraInfo, self.config['info_topic'], self._info_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, self.config['image_topic'], self._image_cb,
            qos_profile_sensor_data)
        self.create_timer(1.0, self._timeout_check)
        self.get_logger().info(
            f'Capturing {self.requested_samples} samples for {self.camera} '
            f'into {self.session_dir}')

    def _info_cb(self, msg: CameraInfo) -> None:
        self.camera_info = msg

    def _lookup(self, target: str, source: str):
        try:
            msg = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
            return transform_from_msg(msg.transform)
        except Exception:
            return None

    def _timeout_check(self) -> None:
        if self.done:
            self.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            return
        if time.monotonic() - self.started > self.max_seconds:
            self.get_logger().warn('Calibration capture timeout')
            self._finish()

    def _tool_pose(self):
        if self.config['mode'] != 'eye_in_hand':
            return np.eye(4)
        return self._lookup(self.config['base_frame'], self.config['tool_frame'])

    def _tool_is_new(self, pose: np.ndarray) -> bool:
        if self.last_tool_pose is None:
            return True
        delta = invert_transform(self.last_tool_pose) @ pose
        translation = float(np.linalg.norm(delta[:3, 3]))
        rotation = float(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec()))
        return (
            translation >= self.min_tool_motion or
            np.degrees(rotation) >= self.min_tool_rotation
        )

    def _detect_board(self, gray, K, D):
        if self.board_type == 'chessboard':
            flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            ok, corners = cv2.findChessboardCornersSB(
                gray, self.pattern_size, flags=flags)
            if not ok:
                flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                         cv2.CALIB_CB_NORMALIZE_IMAGE)
                ok, corners = cv2.findChessboardCorners(
                    gray, self.pattern_size, flags=flags)
            if not ok or corners is None:
                return None
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                        50, 0.001)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
            image_points = corners.reshape(-1, 2)
            object_points = self.chessboard_object_points
            ok, rvec, tvec = cv2.solvePnP(
                object_points, image_points, K, D,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok:
                return None
            try:
                rvec, tvec = cv2.solvePnPRefineLM(
                    object_points, image_points, K, D, rvec, tvec)
            except Exception:
                pass
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary)
            if ids is None or len(ids) == 0:
                return None
            count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, self.board)
            if (charuco_corners is None or charuco_ids is None or
                    count < self.min_corners):
                return None
            ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board, K, D, None, None)
            if not ok:
                return None
            ids_flat = charuco_ids.reshape(-1)
            board_corners = np.asarray(self.board.chessboardCorners).reshape(-1, 3)
            object_points = board_corners[ids_flat]
            image_points = charuco_corners.reshape(-1, 2)

        R_cam_board = Rotation.from_rotvec(rvec.reshape(3)).as_matrix()
        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = R_cam_board
        T_cam_board[:3, 3] = tvec.reshape(3)
        return object_points, image_points, T_cam_board

    def _image_cb(self, msg: Image) -> None:
        if self.done or self.camera_info is None:
            return
        now = time.monotonic()
        if now - self.last_capture < self.min_interval:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'image conversion failed: {exc}')
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        K = np.asarray(self.camera_info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(self.camera_info.d, dtype=np.float64)
        detected = self._detect_board(gray, K, D)
        if detected is None:
            return
        object_points, image_points, T_cam_board = detected

        tool_pose = self._tool_pose()
        if tool_pose is None:
            self.get_logger().warn('Robot tool TF unavailable; sample skipped')
            return
        if self.config['mode'] == 'eye_in_hand' and not self._tool_is_new(tool_pose):
            return
        camera_link_optical = self._lookup(
            self.config['camera_link_frame'], self.config['optical_frame'])
        if camera_link_optical is None:
            self.get_logger().warn('Camera link/optical TF unavailable')
            return

        sample_path = self.session_dir / f'sample_{self.sample_count:04d}.npz'
        np.savez(
            sample_path,
            camera_matrix=K,
            distortion=D,
            image_size=np.array([msg.width, msg.height], dtype=np.int32),
            object_points=object_points,
            image_points=image_points,
            T_cam_board=T_cam_board,
            T_base_tool=tool_pose,
            T_camera_link_optical=camera_link_optical,
            stamp_sec=int(msg.header.stamp.sec),
            stamp_nanosec=int(msg.header.stamp.nanosec),
        )
        self.saved_meta.append(str(sample_path.relative_to(self.session_dir)))
        self.sample_count += 1
        self.last_capture = now
        self.last_tool_pose = tool_pose.copy()
        self.get_logger().info(
            f'captured {self.sample_count}/{self.requested_samples}: {sample_path.name}')
        if self.sample_count >= self.requested_samples:
            self._finish()

    def _finish(self) -> None:
        if self.done:
            return
        self.done = True
        metadata = {
            'camera': self.camera,
            'mode': self.config['mode'],
            'image_topic': self.config['image_topic'],
            'info_topic': self.config['info_topic'],
            'camera_link_frame': self.config['camera_link_frame'],
            'optical_frame': self.config['optical_frame'],
            'parent_frame': self.config['parent_frame'],
            'base_frame': self.config.get('base_frame'),
            'tool_frame': self.config.get('tool_frame'),
            'board': self.board_cfg,
            'samples': self.saved_meta,
        }
        with (self.session_dir / 'metadata.yaml').open('w', encoding='utf-8') as stream:
            yaml.safe_dump(metadata, stream, sort_keys=False)
        self.get_logger().info(f'capture finished: {self.session_dir}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CharucoCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
