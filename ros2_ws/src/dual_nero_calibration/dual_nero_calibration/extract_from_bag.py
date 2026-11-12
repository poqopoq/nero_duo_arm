#!/usr/bin/env python3
"""Select calibration samples offline from a ROS 2 bag."""
from __future__ import annotations

import argparse
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer

from .board_quality import DetectionError, detect_chessboard
from .common import (
    default_deployment_dir,
    invert_transform,
    load_yaml,
    transform_from_msg,
)
from .frame_selection import select_diverse_indices
from .solve_extrinsics import max_rotation_spread_deg


@dataclass
class Candidate:
    stamp_ns: int
    detection: object
    T_base_tool: np.ndarray
    T_camera_link_optical: np.ndarray
    camera_matrix: np.ndarray
    distortion: np.ndarray
    image_size: np.ndarray


def _reader(bag: Path, topics: list[str]) -> SequentialReader:
    metadata = load_yaml(bag / 'metadata.yaml')['rosbag2_bagfile_information']
    storage_id = metadata.get('storage_identifier', 'sqlite3')
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id=storage_id),
        ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    if topics:
        reader.set_filter(StorageFilter(topics=topics))
    return reader


def _topics(reader: SequentialReader) -> dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def _message_types(topics: dict[str, str], names: list[str]) -> dict[str, type]:
    return {name: get_message(topics[name]) for name in names if name in topics}


def load_tf(bag: Path, tf_topic: str, tf_static_topic: str) -> tuple[Buffer, int]:
    probe = _reader(bag, [])
    topics = _topics(probe)
    selected = [name for name in (tf_topic, tf_static_topic) if name in topics]
    if not selected:
        return Buffer(cache_time=Duration(seconds=7 * 24 * 3600)), 0

    reader = _reader(bag, selected)
    message_types = _message_types(topics, selected)
    buffer = Buffer(cache_time=Duration(seconds=7 * 24 * 3600))
    count = 0
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, message_types[topic])
        for transform in msg.transforms:
            if topic == tf_static_topic:
                buffer.set_transform_static(transform, 'bag')
            else:
                buffer.set_transform(transform, 'bag')
            count += 1
    return buffer, count


def load_camera_infos(bag: Path, topic: str) -> list[tuple[int, object]]:
    probe = _reader(bag, [])
    topics = _topics(probe)
    if topic not in topics:
        available = ', '.join(sorted(topics))
        raise RuntimeError(
            f'CameraInfo topic {topic!r} is not in the bag. Available: {available}')
    reader = _reader(bag, [topic])
    message_type = get_message(topics[topic])
    infos = []
    while reader.has_next():
        _, data, bag_stamp = reader.read_next()
        msg = deserialize_message(data, message_type)
        stamp_ns = _stamp_ns(msg.header.stamp, bag_stamp)
        infos.append((stamp_ns, msg))
    infos.sort(key=lambda item: item[0])
    return infos


def _stamp_ns(stamp, fallback_ns: int) -> int:
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback_ns)


def _nearest_info(infos, stamp_ns: int, tolerance_ns: int = 200_000_000):
    if not infos:
        return None
    times = [item[0] for item in infos]
    index = bisect_left(times, stamp_ns)
    choices = []
    if index > 0:
        choices.append(infos[index - 1])
    if index < len(infos):
        choices.append(infos[index])
    best = min(choices, key=lambda item: abs(item[0] - stamp_ns))
    if abs(best[0] - stamp_ns) > tolerance_ns:
        return None
    return best[1]


def _lookup(buffer: Buffer, target: str, source: str, stamp_ns: int):
    try:
        msg = buffer.lookup_transform(
            target, source, Time(nanoseconds=int(stamp_ns)))
        return transform_from_msg(msg.transform)
    except Exception:
        return None


def _tool_is_stationary(buffer: Buffer, camera_cfg, T_base_tool: np.ndarray,
                        stamp_ns: int, window_sec: float,
                        max_translation_m: float,
                        max_rotation_deg: float) -> bool:
    window_ns = int(window_sec * 1_000_000_000)
    for offset in (-window_ns, window_ns):
        other = _lookup(
            buffer, camera_cfg['base_frame'], camera_cfg['tool_frame'],
            stamp_ns + offset)
        if other is None:
            return False
        delta = invert_transform(T_base_tool) @ other
        translation_m = float(np.linalg.norm(delta[:3, 3]))
        rotation_deg = float(np.degrees(np.linalg.norm(
            Rotation.from_matrix(delta[:3, :3]).as_rotvec())))
        if (translation_m > max_translation_m or
                rotation_deg > max_rotation_deg):
            return False
    return True


def collect_candidates(args, camera_cfg, board_cfg, pattern_size,
                       object_points, infos, tf_buffer) -> tuple[list[Candidate], dict]:
    bag = Path(args.bag).expanduser().resolve()
    probe = _reader(bag, [])
    topics = _topics(probe)
    image_topic = camera_cfg['image_topic']
    if image_topic not in topics:
        available = ', '.join(sorted(topics))
        raise RuntimeError(
            f'Image topic {image_topic!r} is not in the bag. Available: {available}')

    reader = _reader(bag, [image_topic])
    image_type = get_message(topics[image_topic])
    bridge = CvBridge()
    candidates = []
    stats = {
        'images': 0,
        'no_camera_info': 0,
        'no_tool_tf': 0,
        'no_camera_tf': 0,
        'not_stationary': 0,
        'no_board': 0,
        'bad_quality': 0,
    }

    frame_index = 0
    while reader.has_next():
        _, data, bag_stamp = reader.read_next()
        if frame_index % max(1, args.stride) != 0:
            frame_index += 1
            continue
        frame_index += 1
        stats['images'] += 1

        msg = deserialize_message(data, image_type)
        stamp_ns = _stamp_ns(msg.header.stamp, bag_stamp)
        info = _nearest_info(infos, stamp_ns)
        if info is None:
            stats['no_camera_info'] += 1
            continue

        if camera_cfg['mode'] == 'eye_in_hand':
            T_base_tool = _lookup(
                tf_buffer, camera_cfg['base_frame'], camera_cfg['tool_frame'],
                stamp_ns)
            if T_base_tool is None:
                stats['no_tool_tf'] += 1
                continue
            if (not args.allow_moving_frames and
                    not _tool_is_stationary(
                        tf_buffer, camera_cfg, T_base_tool, stamp_ns,
                        args.stationary_window_sec,
                        args.max_stationary_translation_mm / 1000.0,
                        args.max_stationary_rotation_deg)):
                stats['not_stationary'] += 1
                continue
        else:
            T_base_tool = np.eye(4)

        T_link_optical = _lookup(
            tf_buffer, camera_cfg['camera_link_frame'],
            camera_cfg['optical_frame'], stamp_ns)
        if T_link_optical is None:
            stats['no_camera_tf'] += 1
            continue

        try:
            image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            stats['no_board'] += 1
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        K = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(info.d, dtype=np.float64)
        try:
            detection = detect_chessboard(
                gray, pattern_size, object_points, K, D,
                min_corner_gap_px=args.min_corner_gap_px,
                max_reprojection_rms_px=args.max_reprojection_rms_px)
        except DetectionError:
            stats['bad_quality'] += 1
            continue

        candidates.append(Candidate(
            stamp_ns=stamp_ns,
            detection=detection,
            T_base_tool=T_base_tool,
            T_camera_link_optical=T_link_optical,
            camera_matrix=K,
            distortion=D,
            image_size=np.array([msg.width, msg.height], dtype=np.int32),
        ))
        if (args.progress_every > 0 and
                stats['images'] % args.progress_every == 0):
            print(
                f"progress images={stats['images']} "
                f"candidates={len(candidates)}", flush=True)
    return candidates, stats


def select_candidates(candidates: list[Candidate], camera_cfg,
                      count: int, translation_weight: float) -> list[Candidate]:
    if camera_cfg['mode'] == 'eye_in_hand':
        poses = [candidate.T_base_tool for candidate in candidates]
    else:
        poses = [candidate.detection.T_cam_board for candidate in candidates]
    indices = select_diverse_indices(
        rotations=[pose[:3, :3] for pose in poses],
        translations=[pose[:3, 3] for pose in poses],
        qualities=[candidate.detection.reprojection_rms_px
                   for candidate in candidates],
        count=count,
        translation_weight=translation_weight,
    )
    selected = [candidates[index] for index in indices]
    return sorted(selected, key=lambda candidate: candidate.stamp_ns)


def save_session(args, camera_cfg, board_cfg, selected, stats):
    output_root = (
        Path(args.output_dir).expanduser()
        if args.output_dir else default_deployment_dir('calibration_data'))
    bag_name = Path(args.bag).expanduser().resolve().name
    bag_name = ''.join(
        character if character.isalnum() or character in '-_' else '_'
        for character in bag_name)
    session = output_root / args.camera / (
        f'bag_{bag_name}_{time.strftime("%Y%m%d_%H%M%S")}')
    session.mkdir(parents=True, exist_ok=True)

    saved = []
    for index, candidate in enumerate(selected):
        path = session / f'sample_{index:04d}.npz'
        detection = candidate.detection
        np.savez(
            path,
            camera_matrix=candidate.camera_matrix,
            distortion=candidate.distortion,
            image_size=candidate.image_size,
            object_points=detection.object_points,
            image_points=detection.image_points,
            T_cam_board=detection.T_cam_board,
            T_base_tool=candidate.T_base_tool,
            T_camera_link_optical=candidate.T_camera_link_optical,
            reprojection_rms_px=detection.reprojection_rms_px,
            stamp_sec=int(candidate.stamp_ns // 1_000_000_000),
            stamp_nanosec=int(candidate.stamp_ns % 1_000_000_000),
        )
        saved.append({
            'file': path.name,
            'stamp_ns': int(candidate.stamp_ns),
            'reprojection_rms_px': float(detection.reprojection_rms_px),
            'min_corner_gap_px': float(detection.min_corner_gap_px),
        })

    metadata = {
        'camera': args.camera,
        'mode': camera_cfg['mode'],
        'source_bag': str(Path(args.bag).expanduser().resolve()),
        'image_topic': camera_cfg['image_topic'],
        'info_topic': camera_cfg['info_topic'],
        'camera_link_frame': camera_cfg['camera_link_frame'],
        'optical_frame': camera_cfg['optical_frame'],
        'parent_frame': camera_cfg['parent_frame'],
        'base_frame': camera_cfg.get('base_frame'),
        'tool_frame': camera_cfg.get('tool_frame'),
        'board': board_cfg,
        'selection': {
            'strategy': 'quality_gate_then_farthest_pose_sampling',
            'requested_samples': int(args.samples),
            'stride': int(args.stride),
            'translation_weight': float(args.translation_weight),
            'max_reprojection_rms_px': float(args.max_reprojection_rms_px),
            'min_corner_gap_px': float(args.min_corner_gap_px),
            'stationary_check': not bool(args.allow_moving_frames),
            'stationary_window_sec': float(args.stationary_window_sec),
            'max_stationary_translation_mm': float(
                args.max_stationary_translation_mm),
            'max_stationary_rotation_deg': float(
                args.max_stationary_rotation_deg),
            'stats': stats,
        },
        'samples': saved,
    }
    with (session / 'metadata.yaml').open('w', encoding='utf-8') as stream:
        yaml.safe_dump(metadata, stream, sort_keys=False)
    return session


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', required=True,
                        choices=('scene', 'left_wrist', 'right_wrist'))
    parser.add_argument('--bag', required=True,
                        help='ROS 2 bag directory containing image, CameraInfo and /tf')
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--output-dir', default='')
    parser.add_argument('--image-topic', default='',
                        help='Override the camera image topic')
    parser.add_argument('--info-topic', default='',
                        help='Override the CameraInfo topic')
    parser.add_argument('--tf-topic', default='/tf')
    parser.add_argument('--tf-static-topic', default='/tf_static')
    parser.add_argument('--stride', type=int, default=1,
                        help='Inspect every Nth image (default: 1)')
    parser.add_argument('--max-reprojection-rms-px', type=float, default=1.0)
    parser.add_argument('--min-corner-gap-px', type=float, default=2.0)
    parser.add_argument('--stationary-window-sec', type=float, default=0.15,
                        help='Half-window used to verify the robot is stopped')
    parser.add_argument('--max-stationary-translation-mm', type=float,
                        default=2.0)
    parser.add_argument('--max-stationary-rotation-deg', type=float,
                        default=0.3)
    parser.add_argument('--allow-moving-frames', action='store_true',
                        help='Disable the stationary gate (not recommended)')
    parser.add_argument('--progress-every', type=int, default=200,
                        help='Print progress every N inspected images')
    parser.add_argument('--min-samples', type=int, default=6)
    parser.add_argument('--translation-weight', type=float, default=500.0,
                        help='Pose-distance weight: metres to equivalent degrees')
    parser.add_argument('--strict', action='store_true',
                        help='Return non-zero when fewer than --min-samples qualify')
    args = parser.parse_args()

    bag = Path(args.bag).expanduser().resolve()
    if not (bag / 'metadata.yaml').is_file():
        print(f'ERROR not a ROS 2 bag directory: {bag}', file=sys.stderr)
        return 2

    share = Path(get_package_share_directory('dual_nero_calibration'))
    targets = load_yaml(share / 'config' / 'calibration_targets.yaml')['cameras']
    camera_cfg = dict(targets[args.camera])
    if args.image_topic:
        camera_cfg['image_topic'] = args.image_topic
    if args.info_topic:
        camera_cfg['info_topic'] = args.info_topic
    board_cfg = load_yaml(share / 'config' / 'calibration_board.yaml')['board']
    if board_cfg.get('type', 'chessboard') != 'chessboard':
        print('ERROR offline selection currently supports chessboard only',
              file=sys.stderr)
        return 2

    pattern_size = (int(board_cfg['pattern_cols']), int(board_cfg['pattern_rows']))
    square = float(board_cfg['square_length_m'])
    grid = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    object_points = np.column_stack(
        [grid.astype(np.float32) * square,
         np.zeros(len(grid), dtype=np.float32)])

    rclpy.init()
    try:
        tf_buffer, tf_count = load_tf(
            bag, args.tf_topic, args.tf_static_topic)
        if tf_count == 0:
            print(
                'ERROR no /tf or /tf_static in the bag; wrist hand-eye needs '
                'robot poses', file=sys.stderr)
            return 2
        infos = load_camera_infos(bag, camera_cfg['info_topic'])
        candidates, stats = collect_candidates(
            args, camera_cfg, board_cfg, pattern_size, object_points, infos,
            tf_buffer)
    finally:
        rclpy.shutdown()

    print('CANDIDATES', len(candidates))
    print('STATS', stats)
    if not candidates:
        print('ERROR no valid chessboard frames', file=sys.stderr)
        return 2

    selected = select_candidates(
        candidates, camera_cfg, args.samples, args.translation_weight)
    session = save_session(args, camera_cfg, board_cfg, selected, stats)

    if camera_cfg['mode'] == 'eye_in_hand':
        poses = [candidate.T_base_tool for candidate in selected]
    else:
        poses = [candidate.detection.T_cam_board for candidate in selected]
    spread = max_rotation_spread_deg([pose[:3, :3] for pose in poses])
    rms_values = [candidate.detection.reprojection_rms_px
                  for candidate in selected]
    print(f'SESSION {session}')
    print('SELECTED', len(selected))
    print('ROTATION_SPREAD_DEG', round(spread, 6))
    print('REPROJECTION_RMS_PX_MAX', round(max(rms_values), 6))
    passed = len(selected) >= args.min_samples
    print('SELECTION', 'PASS' if passed else 'WARN')
    if args.strict and not passed:
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
