import numpy as np
from contact_graspnet_ros.core.geometry import (
    build_6d_grasp_numpy,
    depth_to_point_cloud,
    distance_by_translation_point,
    extract_point_clouds,
    farthest_points,
    regularize_pc_point_count,
    select_grasps,
)


def test_depth_to_point_cloud_known_values() -> None:
    depth = np.array([[2.0, 0.0], [0.0, 2.0]], dtype=np.float32)
    K = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)

    points = depth_to_point_cloud(depth, K)

    assert points.shape == (2, 3)
    assert np.allclose(points[0], [0.0, 0.0, 2.0])
    assert np.allclose(points[1], [2.0, 2.0, 2.0])


def test_extract_point_clouds_and_z_range() -> None:
    depth = np.full((2, 2), 2.0, dtype=np.float32)
    K = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    mask = np.array([[1, 1], [0, 2]], dtype=np.int32)

    pc_full, segments = extract_point_clouds(depth, K, mask, z_range=(1.0, 3.0))

    assert pc_full.shape == (4, 3)
    assert set(segments) == {1, 2}
    assert segments[1].shape[0] == 2
    assert segments[2].shape[0] == 1

    full_clipped, clipped_segments = extract_point_clouds(
        depth, K, mask, z_range=(0.0, 1.0)
    )
    assert full_clipped.shape == (0, 3)
    assert clipped_segments == {}


def test_regularize_pc_point_count_round_trips() -> None:
    rng = np.random.default_rng(0)
    points = rng.random((10, 3), dtype=np.float32)

    downsampled = regularize_pc_point_count(points, 4, rng=rng)
    upsampled = regularize_pc_point_count(points, 25, rng=rng)

    assert downsampled.shape == (4, 3)
    assert upsampled.shape == (25, 3)
    assert downsampled.dtype == np.float32
    assert upsampled.dtype == np.float32


def test_farthest_points_uses_matching_xyz_shapes() -> None:
    points = np.array(
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )
    _, centers = farthest_points(
        points, 2, distance_by_translation_point, return_center_indexes=True
    )
    assert centers.shape == (2,)
    assert len(set(centers.tolist())) == 2


def test_select_grasps_keeps_single_index_zero() -> None:
    contacts = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
    scores = np.array([0.9], dtype=np.float32)

    selection = select_grasps(contacts, scores, first_thres=0.5, second_thres=0.1)

    assert selection.tolist() == [0]


def test_build_6d_grasp_pose_convention() -> None:
    grasps = build_6d_grasp_numpy(
        approach_dirs=np.array([[0.0, 0.0, 1.0]], dtype=np.float32),
        base_dirs=np.array([[1.0, 0.0, 0.0]], dtype=np.float32),
        contact_pts=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
        thickness=np.array([0.08], dtype=np.float32),
    )

    grasp = grasps[0]
    assert np.allclose(grasp[:3, 0], [1.0, 0.0, 0.0])
    assert np.allclose(grasp[:3, 1], [0.0, 1.0, 0.0])
    assert np.allclose(grasp[:3, 2], [0.0, 0.0, 1.0])
    assert np.allclose(grasp[:3, 3], [0.04, 0.0, -0.1034])
