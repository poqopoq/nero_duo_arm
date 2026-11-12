import cv2
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from dual_nero_calibration.board_quality import (
    DetectionError,
    detect_chessboard,
    min_adjacent_gap_px,
    reprojection_rms,
)
from dual_nero_calibration.common import (
    invert_transform,
)
from dual_nero_calibration.gripper_tcp import (
    DEFAULT_TCP_RPY,
    driver_offset,
    fit_opening_map,
    solve_pivot,
    urdf_tcp_origin,
)
from dual_nero_calibration.frame_selection import select_diverse_indices
from dual_nero_calibration.solve_extrinsics import (
    solve_eye_in_hand,
    solve_fixed,
)


def _transform(rotation: Rotation, translation) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation.as_matrix()
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def test_eye_in_hand_recovers_tool_to_camera_transform():
    rng = np.random.default_rng(7)
    tool_camera_link = _transform(
        Rotation.from_euler('xyz', [0.35, -0.42, 1.15]),
        [0.045, -0.018, 0.072],
    )
    link_optical = _transform(
        Rotation.from_euler('xyz', [-np.pi / 2, 0.0, -np.pi / 2]),
        [0.0, 0.0, 0.0],
    )
    tool_camera_optical = tool_camera_link @ link_optical
    base_board = _transform(
        Rotation.from_euler('xyz', [0.1, -0.2, 0.3]),
        [0.55, -0.12, 0.08],
    )

    samples = []
    for _ in range(12):
        base_tool = _transform(
            Rotation.from_euler('xyz', rng.uniform(-0.65, 0.65, 3)),
            rng.uniform([-0.08, -0.08, 0.16], [0.08, 0.08, 0.36]),
        )
        cam_board = (
            invert_transform(tool_camera_optical)
            @ invert_transform(base_tool)
            @ base_board
        )
        samples.append({
            'T_base_tool': base_tool,
            'T_cam_board': cam_board,
            'T_camera_link_optical': link_optical,
        })

    camera_cfg = {
        'parent_frame': 'tool0',
        'camera_link_frame': 'camera_link',
    }
    parent, child, transform, trans_rms, rot_rms = solve_eye_in_hand(
        samples, camera_cfg)

    assert parent == 'tool0'
    assert child == 'camera_link'
    np.testing.assert_allclose(
        transform, tool_camera_link, atol=1e-7, rtol=0.0)
    assert trans_rms < 1e-7
    assert rot_rms < 1e-6


def test_fixed_camera_recovers_world_to_camera_transform():
    world_camera_link = _transform(
        Rotation.from_euler('xyz', [1.35, -0.22, 1.57]),
        [0.62, -0.06, 0.18],
    )
    link_optical = _transform(
        Rotation.from_euler('xyz', [-np.pi / 2, 0.0, -np.pi / 2]),
        [0.0, 0.0, 0.0],
    )
    world_camera_optical = world_camera_link @ link_optical
    world_board = _transform(
        Rotation.from_euler('xyz', [0.0, 0.0, 0.25]),
        [0.45, 0.0, 0.0],
    )
    cam_board = invert_transform(world_camera_optical) @ world_board
    samples = [{
        'T_cam_board': cam_board,
        'T_camera_link_optical': link_optical,
    } for _ in range(5)]
    camera_cfg = {
        'mode': 'fixed',
        'parent_frame': 'world',
        'camera_link_frame': 'camera_scene_link',
        'board_pose_in_world': {'xyz': [0.45, 0.0, 0.0],
                                'rpy': [0.0, 0.0, 0.25]},
    }

    parent, child, transform, trans_rms, rot_rms = solve_fixed(
        samples, camera_cfg)

    assert parent == 'world'
    assert child == 'camera_scene_link'
    np.testing.assert_allclose(
        transform, world_camera_link, atol=1e-9, rtol=0.0)
    assert trans_rms < 1e-9
    assert rot_rms < 1e-8


def test_pivot_recovers_tcp_translation():
    rng = np.random.default_rng(11)
    tcp = np.array([0.2443, 0.0, -0.0235])
    pivot = np.array([0.42, -0.11, 0.19])
    transforms = []
    for _ in range(12):
        rotation = Rotation.from_euler('xyz', rng.uniform(-0.8, 0.8, 3))
        transform = _transform(rotation, [0.0, 0.0, 0.0])
        transform[:3, 3] = pivot - transform[:3, :3] @ tcp
        transforms.append(transform)

    result = solve_pivot(transforms)

    np.testing.assert_allclose(result.tcp_xyz_link7, tcp, atol=1e-10, rtol=0.0)
    np.testing.assert_allclose(
        result.pivot_xyz_base, pivot, atol=1e-10, rtol=0.0)
    assert result.rms_m < 1e-10
    assert result.rotation_spread_deg > 20.0


def test_pivot_rejects_degenerate_orientation_set():
    transform = _transform(Rotation.from_euler('xyz', [0.0, 0.0, 0.0]),
                           [0.1, 0.2, 0.3])
    with pytest.raises(ValueError, match='degenerate'):
        solve_pivot([transform.copy() for _ in range(5)])


def test_default_tcp_matches_urdf_and_driver_contract():
    tcp_link7 = np.array([0.2443, 0.0, -0.0235])
    offset = driver_offset(tcp_link7, DEFAULT_TCP_RPY)
    np.testing.assert_allclose(offset[:3, 3], tcp_link7, atol=1e-12)
    urdf = urdf_tcp_origin(tcp_link7, DEFAULT_TCP_RPY)
    np.testing.assert_allclose(urdf[:3, 3], [0.0, 0.0, 0.2133], atol=1e-6)
    np.testing.assert_allclose(urdf[:3, :3], np.eye(3), atol=1e-6)


def test_opening_map_fit():
    measurements = [(0, 90.0), (500, 45.0), (900, 9.0)]
    result = fit_opening_map(measurements)
    assert result['slope_mm_per_count'] == pytest.approx(-0.09)
    assert result['opening_min_mm'] == pytest.approx(90.0)
    assert result['opening_max_mm'] == pytest.approx(0.0)
    assert result['rms_mm'] == pytest.approx(0.0, abs=1e-12)


def test_min_adjacent_gap_detects_collapsed_corner():
    pattern_size = (11, 8)
    xs, ys = np.meshgrid(np.arange(11) * 10.0, np.arange(8) * 10.0)
    clean = np.column_stack([xs.ravel(), ys.ravel()])
    assert min_adjacent_gap_px(clean, pattern_size) == pytest.approx(10.0)

    collapsed = clean.copy()
    collapsed[1] = collapsed[0]
    assert min_adjacent_gap_px(collapsed, pattern_size) == pytest.approx(0.0)


def test_min_adjacent_gap_rejects_wrong_corner_count():
    with pytest.raises(ValueError, match='expected 88 corners'):
        min_adjacent_gap_px(np.zeros((10, 2)), (11, 8))


def test_reprojection_rms_reports_bad_corner():
    pattern_size = (11, 8)
    square = 0.015
    xs, ys = np.meshgrid(np.arange(11) * square, np.arange(8) * square)
    object_points = np.column_stack(
        [xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    camera_matrix = np.array([
        [500.0, 0.0, 320.0],
        [0.0, 500.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    distortion = np.zeros(5)
    rvec = np.array([0.3, -0.2, 0.05])
    tvec = np.array([-0.08, 0.04, 0.4])
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion)
    image_points = projected.reshape(-1, 2)

    assert reprojection_rms(
        object_points, image_points, camera_matrix, distortion) < 1e-9

    corrupted = image_points.copy()
    corrupted[17] += [6.0, -6.0]
    assert reprojection_rms(
        object_points, corrupted, camera_matrix, distortion) > 0.8


def test_detect_chessboard_accepts_synthetic_board():
    cols, rows, square_px = 11, 8, 30
    image = np.full(
        ((rows + 3) * square_px, (cols + 3) * square_px), 255, np.uint8)
    for row in range(rows + 1):
        for col in range(cols + 1):
            if (row + col) % 2 == 0:
                image[(row + 1) * square_px:(row + 2) * square_px,
                      (col + 1) * square_px:(col + 2) * square_px] = 0

    object_points = np.column_stack([
        np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * 0.015,
        np.zeros(cols * rows),
    ])
    camera_matrix = np.array([
        [900.0, 0.0, image.shape[1] / 2],
        [0.0, 900.0, image.shape[0] / 2],
        [0.0, 0.0, 1.0],
    ])
    detection = detect_chessboard(
        image, (cols, rows), object_points, camera_matrix, np.zeros(5))

    assert detection.image_points.shape == (88, 2)
    assert detection.min_corner_gap_px == pytest.approx(30.0, abs=0.2)
    assert detection.reprojection_rms_px < 0.1
    with pytest.raises(DetectionError, match='reprojection RMS'):
        detect_chessboard(
            image, (cols, rows), object_points, camera_matrix, np.zeros(5),
            max_reprojection_rms_px=1e-6)


def test_frame_selection_skips_near_duplicate_pose():
    def rotation(degrees):
        return Rotation.from_euler('z', degrees, degrees=True).as_matrix()

    rotations = [
        rotation(0.0), rotation(0.1), rotation(30.0), rotation(-30.0)]
    translations = np.array([
        [0.0, 0.0, 0.0],
        [0.0001, 0.0, 0.0],
        [0.1, 0.0, 0.0],
        [-0.1, 0.0, 0.0],
    ])
    qualities = np.array([0.1, 0.2, 0.5, 0.6])

    selected = select_diverse_indices(
        rotations, translations, qualities, count=3)

    assert len(selected) == 3
    assert selected[0] == 0
    assert 1 not in selected
    assert set(selected[1:]) == {2, 3}


def test_frame_selection_handles_empty_and_oversized_request():
    assert select_diverse_indices([], [], [], count=5) == []
    rotations = [np.eye(3), Rotation.from_euler('z', 10, degrees=True).as_matrix()]
    translations = [[0.0, 0.0, 0.0], [0.01, 0.0, 0.0]]
    qualities = [1.0, 0.5]
    assert sorted(select_diverse_indices(
        rotations, translations, qualities, count=5)) == [0, 1]


def test_nearest_pose_flags_near_duplicates_only():
    from dual_nero_calibration.frame_selection import nearest_pose

    recorded = [_transform(Rotation.from_euler('z', 0.0), [0.0, 0.0, 0.0])]
    same = _transform(Rotation.from_euler('z', 1.0, degrees=True),
                      [0.002, 0.0, 0.0])
    moved = _transform(Rotation.from_euler('z', 8.0, degrees=True),
                       [0.0, 0.0, 0.0])

    match = nearest_pose(same, recorded, 3.0, 10.0)
    assert match is not None
    assert match[0] == 0
    assert match[1] < 0.01

    assert nearest_pose(moved, recorded, 3.0, 10.0) is None
    assert nearest_pose(same, [], 3.0, 10.0) is None


def test_pose_rotation_spread_deg():
    from dual_nero_calibration.frame_selection import pose_rotation_spread_deg

    poses = [
        _transform(Rotation.from_euler('z', 0.0), [0.0, 0.0, 0.0]),
        _transform(Rotation.from_euler('z', 30.0, degrees=True), [0.0, 0.0, 0.0]),
        _transform(Rotation.from_euler('z', 10.0, degrees=True), [0.0, 0.0, 0.0]),
    ]

    assert pose_rotation_spread_deg(poses) == pytest.approx(30.0, abs=1e-6)
    assert pose_rotation_spread_deg([]) == 0.0


def _write_session(tmp_path, name, tool_camera, base_board, seed):
    from dual_nero_calibration.session import make_session_dir, save_sample

    rng = np.random.default_rng(seed)
    directory = tmp_path / name
    directory.mkdir()
    for index in range(8):
        tool = _transform(
            Rotation.from_euler('xyz', rng.uniform(-0.7, 0.7, 3)),
            rng.uniform(-0.15, 0.15, 3))
        save_sample(
            directory, index,
            camera_matrix=np.array([[400.0, 0.0, 320.0],
                                    [0.0, 400.0, 240.0],
                                    [0.0, 0.0, 1.0]]),
            distortion=np.zeros(5),
            image_size=(640, 480),
            object_points=np.zeros((4, 3)),
            image_points=np.zeros((4, 2)),
            T_cam_board=invert_transform(tool @ tool_camera) @ base_board,
            T_base_tool=tool,
            T_camera_link_optical=np.eye(4),
            reprojection_rms_px=0.1,
            stamp_sec=0,
            stamp_nanosec=0)
    return directory


def test_compare_sessions_passes_for_a_repeatable_setup(tmp_path):
    import types

    from dual_nero_calibration.solve_extrinsics import (
        _compare_sessions,
        load_samples,
        solve_eye_in_hand,
    )

    tool_camera = _transform(Rotation.from_euler('xyz', [0.2, -0.3, 1.2]),
                             [0.07, -0.01, 0.16])
    base_board = _transform(Rotation.from_euler('y', 0.2), [0.5, 0.0, 0.1])
    first = _write_session(tmp_path, 'a', tool_camera, base_board, 1)
    second = _write_session(tmp_path, 'b', tool_camera, base_board, 2)
    cfg = {'parent_frame': 'tool0', 'camera_link_frame': 'cam_link',
           'mode': 'eye_in_hand'}
    samples = load_samples(first)
    _, _, transform, _, _ = solve_eye_in_hand(samples, cfg)
    args = types.SimpleNamespace(method='PARK', max_agreement_mm=3.0,
                                 max_agreement_deg=0.3)

    agreement = _compare_sessions(samples, second, cfg, transform, args)

    assert agreement['ok'], agreement
    assert agreement['translation_mm'] < 0.5
    assert agreement['rotation_deg'] < 0.1


def test_compare_sessions_flags_a_moved_camera(tmp_path):
    import types

    from dual_nero_calibration.solve_extrinsics import (
        _compare_sessions,
        load_samples,
        solve_eye_in_hand,
    )

    tool_camera = _transform(Rotation.from_euler('xyz', [0.2, -0.3, 1.2]),
                             [0.07, -0.01, 0.16])
    moved_rotation = (Rotation.from_euler('xyz', [0.2, -0.3, 1.2])
                      * Rotation.from_euler('x', 1.5, degrees=True))
    moved = _transform(moved_rotation, [0.07, -0.01, 0.16])
    base_board = _transform(Rotation.from_euler('y', 0.2), [0.5, 0.0, 0.1])
    first = _write_session(tmp_path, 'a', tool_camera, base_board, 1)
    second = _write_session(tmp_path, 'b', moved, base_board, 2)
    cfg = {'parent_frame': 'tool0', 'camera_link_frame': 'cam_link',
           'mode': 'eye_in_hand'}
    samples = load_samples(first)
    _, _, transform, _, _ = solve_eye_in_hand(samples, cfg)
    args = types.SimpleNamespace(method='PARK', max_agreement_mm=3.0,
                                 max_agreement_deg=0.3)

    agreement = _compare_sessions(samples, second, cfg, transform, args)

    assert not agreement['ok']
    assert agreement['rotation_deg'] > 1.0


def test_extrinsic_yaml_records_quality_status(tmp_path):
    from dual_nero_calibration.common import load_yaml, write_extrinsic_yaml

    path = tmp_path / 'right_wrist_extrinsic.warn.yaml'
    write_extrinsic_yaml(
        path, 'right_wrist', 'right_tool0', 'camera_right_wrist_link',
        np.eye(4), 20, 0.0094, 0.94,
        status='WARN', failed_checks=['translation_rms', 'rotation_rms'])

    data = load_yaml(path)['extrinsic']
    assert data['status'] == 'WARN'
    assert data['failed_checks'] == ['translation_rms', 'rotation_rms']


def test_extrinsic_yaml_defaults_to_pass(tmp_path):
    from dual_nero_calibration.common import load_yaml, write_extrinsic_yaml

    path = tmp_path / 'extrinsic.yaml'
    write_extrinsic_yaml(path, 'scene', 'world', 'camera_scene_link', np.eye(4), 9, 0.001, 0.1)

    data = load_yaml(path)['extrinsic']
    assert data['status'] == 'PASS'
    assert data['failed_checks'] == []


def test_publisher_status_classification():
    from dual_nero_calibration.publish_extrinsic import _calibration_status

    assert _calibration_status({'status': 'PASS'}) == 'PASS'
    assert _calibration_status({'status': 'warn'}) == 'WARN'
    assert _calibration_status({}) == 'UNKNOWN'


def _synthetic_eye_in_hand(samples_count, tool_camera, base_board):
    rng = np.random.default_rng(11)
    samples = []
    for _ in range(samples_count):
        tool = _transform(
            Rotation.from_euler('xyz', rng.uniform(-0.6, 0.6, 3)),
            rng.uniform(-0.2, 0.2, 3))
        cam_board = invert_transform(tool @ tool_camera) @ base_board
        samples.append({'T_base_tool': tool, 'T_cam_board': cam_board})
    return samples


def _robust_samples(count, tool_camera, base_board, seed=17):
    rng = np.random.default_rng(seed)
    samples = []
    for _ in range(count):
        tool = _transform(
            Rotation.from_euler('xyz', rng.uniform(-0.65, 0.65, 3)),
            rng.uniform(-0.2, 0.2, 3))
        samples.append({
            'T_base_tool': tool,
            'T_cam_board': invert_transform(tool @ tool_camera) @ base_board,
            'T_camera_link_optical': np.eye(4),
        })
    return samples


def test_robust_solve_keeps_a_consistent_session():
    from dual_nero_calibration.robust_extrinsic import ransac_eye_in_hand

    tool_camera = _transform(Rotation.from_euler('xyz', [0.2, -0.3, 1.2]),
                             [0.07, -0.01, 0.16])
    base_board = _transform(Rotation.from_euler('y', 0.2), [0.5, 0.0, 0.1])
    samples = _robust_samples(14, tool_camera, base_board)
    cfg = {'parent_frame': 'tool0', 'camera_link_frame': 'cam_link',
           'mode': 'eye_in_hand'}

    result = ransac_eye_in_hand(
        samples, cfg, minimum=8, iterations=200, max_mm=3.0, max_deg=0.5)

    assert result.inlier_fraction == pytest.approx(1.0)
    assert result.spread_deg >= 30.0
    np.testing.assert_allclose(result.transform, tool_camera, atol=1e-6)


def test_robust_solve_rejects_corrupted_frames():
    from dual_nero_calibration.robust_extrinsic import ransac_eye_in_hand

    tool_camera = _transform(Rotation.from_euler('xyz', [0.2, -0.3, 1.2]),
                             [0.07, -0.01, 0.16])
    base_board = _transform(Rotation.from_euler('y', 0.2), [0.5, 0.0, 0.1])
    samples = _robust_samples(14, tool_camera, base_board)
    bad = [2, 7, 11]
    drift = _transform(Rotation.from_euler('x', 4.0, degrees=True),
                       [0.03, 0.0, 0.0])
    for index in bad:
        samples[index]['T_cam_board'] = samples[index]['T_cam_board'] @ drift
    cfg = {'parent_frame': 'tool0', 'camera_link_frame': 'cam_link',
           'mode': 'eye_in_hand'}

    result = ransac_eye_in_hand(
        samples, cfg, minimum=8, iterations=300, max_mm=3.0, max_deg=0.5)

    assert set(result.inliers).isdisjoint(bad)
    assert len(result.inliers) == 11
    np.testing.assert_allclose(result.transform, tool_camera, atol=1e-6)


def test_robust_solve_reports_two_incompatible_models():
    from dual_nero_calibration.robust_extrinsic import ransac_eye_in_hand

    first = _transform(Rotation.from_euler('xyz', [0.2, -0.3, 1.2]),
                       [0.07, -0.01, 0.16])
    second = first @ _transform(Rotation.from_euler('x', 3.0, degrees=True),
                                [0.02, 0.0, 0.0])
    base_board = _transform(Rotation.from_euler('y', 0.2), [0.5, 0.0, 0.1])
    samples = (_robust_samples(6, first, base_board, seed=21)
               + _robust_samples(6, second, base_board, seed=22))
    cfg = {'parent_frame': 'tool0', 'camera_link_frame': 'cam_link',
           'mode': 'eye_in_hand'}

    with pytest.raises(RuntimeError, match='no consistent subset'):
        ransac_eye_in_hand(
            samples, cfg, minimum=8, iterations=200,
            max_mm=3.0, max_deg=0.5)


def test_format_report_is_quiet_for_a_consistent_session():
    from dual_nero_calibration.solve_extrinsics import format_report

    tool_camera = _transform(Rotation.from_euler('xyz', [0.1, -0.2, 1.4]), [0.07, 0.0, 0.16])
    base_board = _transform(Rotation.identity(), [0.5, 0.0, 0.2])
    samples = _synthetic_eye_in_hand(6, tool_camera, base_board)

    report = format_report(samples, tool_camera)

    assert 'same-pose repeatability' in report
    for sample in samples:
        pose = sample['T_base_tool'] @ tool_camera @ sample['T_cam_board']
        np.testing.assert_allclose(pose, base_board, atol=1e-9)


def test_format_report_ratio_near_one_for_pure_orientation_error():
    from dual_nero_calibration.solve_extrinsics import format_report

    tool_camera = _transform(Rotation.from_euler('xyz', [0.1, -0.2, 1.4]), [0.07, 0.0, 0.16])
    base_board = _transform(Rotation.identity(), [0.5, 0.0, 0.2])
    samples = _synthetic_eye_in_hand(8, tool_camera, base_board)
    biased = tool_camera @ _transform(
        Rotation.from_euler('x', 0.5, degrees=True), [0.0, 0.0, 0.0])

    report = format_report(samples, biased)

    ratio_line = [line for line in report.splitlines() if 'median' in line][0]
    median = float(ratio_line.rsplit(' ', 1)[-1])
    assert 0.8 < median < 1.2


def test_pose_groups_finds_repeated_tool_poses():
    from dual_nero_calibration.solve_extrinsics import _pose_groups

    pose = _transform(Rotation.from_euler('xyz', [0.2, 0.1, -0.3]), [0.3, 0.1, 0.2])
    other = _transform(Rotation.from_euler('xyz', [0.2, 0.1, 0.9]), [0.3, 0.1, 0.2])
    samples = [
        {'T_base_tool': pose, 'T_cam_board': np.eye(4)},
        {'T_base_tool': pose @ _transform(Rotation.from_euler('z', 1e-4),
                                          [1e-5, 0.0, 0.0]),
         'T_cam_board': np.eye(4)},
        {'T_base_tool': other, 'T_cam_board': np.eye(4)},
    ]

    lines = '\n'.join(_pose_groups(samples))

    assert 'samples [0, 1]' in lines


class _FakeChain:
    """Three-joint stand-in: yaw, pitch and roll about the same origin."""

    def __init__(self, lower, upper):
        self._lower = np.asarray(lower, dtype=np.float64)
        self._upper = np.asarray(upper, dtype=np.float64)

    def limits(self):
        return self._lower, self._upper

    def forward(self, joints):
        yaw, pitch, roll = np.asarray(joints, dtype=np.float64)
        pose = np.eye(4)
        pose[:3, :3] = (
            Rotation.from_euler('z', yaw).as_matrix()
            @ Rotation.from_euler('y', pitch).as_matrix()
            @ Rotation.from_euler('x', roll).as_matrix())
        pose[:3, 3] = [0.0, 0.0, 0.0]
        return pose


def _planner_setup():
    from dual_nero_calibration.pose_planner import board_corners_in_base

    pattern = (11, 8)
    grid = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * 0.015
    object_points = np.column_stack([grid, np.zeros(len(grid))])
    camera_matrix = np.array([[392.0, 0.0, 311.0],
                              [0.0, 392.0, 232.0],
                              [0.0, 0.0, 1.0]])
    # Board 0.45 m in front of the camera, camera 0.16 m along tool z.
    T_base_board = np.eye(4)
    T_base_board[:3, 3] = [0.0, 0.0, 0.45]
    T_tool_camera = np.eye(4)
    T_tool_camera[:3, 3] = [0.0, 0.0, 0.16]
    corners = board_corners_in_base(T_base_board, object_points)
    return pattern, object_points, camera_matrix, T_base_board, T_tool_camera, corners


def test_view_ok_accepts_board_in_front_and_rejects_off_axis_views():
    from dual_nero_calibration.pose_planner import ViewLimits, view_ok

    pattern, _, camera_matrix, T_base_board, T_tool_camera, corners = _planner_setup()
    limits = ViewLimits(min_gap_px=4.0)

    ok, reason, score = view_ok(T_tool_camera, corners, camera_matrix,
                                (640, 480), pattern, limits)
    assert ok, reason
    assert 0.0 <= score <= 1.0

    behind = np.eye(4)
    behind[:3, 3] = [0.0, 0.0, -0.45]
    ok, _, _ = view_ok(behind, corners, camera_matrix, (640, 480), pattern, limits)
    assert not ok

    turned = T_tool_camera.copy()
    turned[:3, :3] = Rotation.from_euler('y', 80, degrees=True).as_matrix()
    ok, _, _ = view_ok(turned, corners, camera_matrix, (640, 480), pattern, limits)
    assert not ok


def test_view_ok_rejects_board_that_is_too_small_in_the_image():
    from dual_nero_calibration.pose_planner import ViewLimits, view_ok

    pattern, _, camera_matrix, T_base_board, T_tool_camera, corners = _planner_setup()
    far = T_tool_camera.copy()
    far[:3, 3] = [0.0, 0.0, 1.6]

    ok, reason, _ = view_ok(far, corners, camera_matrix, (640, 480), pattern,
                            ViewLimits(min_gap_px=4.0))

    assert not ok


def test_corners_base_to_normal_is_the_board_plane_normal():
    from dual_nero_calibration.pose_planner import corners_base_to_normal

    _, _, _, _, _, corners = _planner_setup()
    normal = corners_base_to_normal(corners)

    np.testing.assert_allclose(normal, [0.0, 0.0, 1.0], atol=1e-9)


def test_generate_candidates_respects_limits_and_keeps_the_board_in_view():
    from dual_nero_calibration.pose_planner import (
        ViewLimits,
        generate_candidates,
        view_ok,
    )

    pattern, object_points, camera_matrix, T_base_board, T_tool_camera, corners = _planner_setup()
    chain = _FakeChain([-0.6, -0.6, -0.6], [0.6, 0.6, 0.6])
    limits = ViewLimits(min_gap_px=4.0)

    candidates = generate_candidates(
        chain=chain, q_seed=np.zeros(3), T_tool_camera=T_tool_camera,
        T_base_board=T_base_board, object_points=object_points,
        camera_matrix=camera_matrix, image_size=(640, 480),
        pattern_size=pattern, count=6, seed=3, max_step_deg=12.0,
        limit_margin_deg=3.0, attempts=2000, limits=limits)

    assert len(candidates) == 6
    lower, upper = chain.limits()
    for candidate in candidates:
        assert np.all(candidate.joints >= lower - 1e-9)
        assert np.all(candidate.joints <= upper + 1e-9)
        ok, reason, _ = view_ok(candidate.T_base_tool @ T_tool_camera,
                                corners, camera_matrix, (640, 480), pattern,
                                limits)
        assert ok, reason
