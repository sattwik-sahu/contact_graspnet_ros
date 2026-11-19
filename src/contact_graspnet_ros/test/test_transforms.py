import numpy as np
from contact_graspnet_ros.core.transforms import (
    apply_transform,
    rotation_matrix_from_quaternion,
    rotation_matrix_from_rpy,
    transform_from_offset_rpy,
    transform_from_xyz_quaternion,
)

HALF_PI = np.pi / 2.0


def test_identity_transform() -> None:
    transform = transform_from_offset_rpy([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    np.testing.assert_allclose(transform, np.eye(4))


def test_yaw_of_90_degrees_maps_x_to_y() -> None:
    rotation = rotation_matrix_from_rpy(0.0, 0.0, HALF_PI)
    np.testing.assert_allclose(
        rotation @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-12
    )
    np.testing.assert_allclose(
        rotation @ np.array([0.0, 1.0, 0.0]), [-1.0, 0.0, 0.0], atol=1e-12
    )
    np.testing.assert_allclose(
        rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0], atol=1e-12
    )


def test_rotation_is_orthonormal() -> None:
    rotation = rotation_matrix_from_rpy(0.3, -0.7, 1.1)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(rotation), 1.0)


def test_quaternion_round_trip() -> None:
    rotation = rotation_matrix_from_rpy(0.2, 0.4, -0.6)
    # Quaternion for the same rotation, w-first ordering avoided on purpose.
    trace = np.trace(rotation)
    s = np.sqrt(trace + 1.0) * 2.0
    quaternion = np.array(
        [
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
            0.25 * s,
        ]
    )
    np.testing.assert_allclose(
        rotation_matrix_from_quaternion(quaternion), rotation, atol=1e-12
    )


def test_transform_from_xyz_quaternion_translates() -> None:
    transform = transform_from_xyz_quaternion(
        [1.0, 2.0, 3.0], [0.0, 0.0, 0.0, 1.0]
    )
    np.testing.assert_allclose(transform[:3, :3], np.eye(3))
    np.testing.assert_allclose(transform[:3, 3], [1.0, 2.0, 3.0])


def test_apply_transform_batch_and_single() -> None:
    transform = transform_from_offset_rpy([0.0, 0.0, 1.0], [0.0, 0.0, HALF_PI])
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])

    transformed = apply_transform(transform, points)
    np.testing.assert_allclose(
        transformed, [[0.0, 1.0, 1.0], [-1.0, 0.0, 1.0]], atol=1e-12
    )

    single = apply_transform(transform, points[0])
    np.testing.assert_allclose(single, transformed[0], atol=1e-12)


def test_pose_composition_matches_manual_formula() -> None:
    # T_target_camera @ T_camera_grasp @ T_grasp_tool
    camera_to_target = transform_from_offset_rpy(
        [0.5, -0.2, 0.9], [0.0, 0.0, HALF_PI]
    )
    grasp = np.eye(4)
    grasp[:3, 3] = [0.1, 0.2, 0.3]
    tool = transform_from_offset_rpy([0.0, 0.0, 0.1034], [0.0, 0.0, HALF_PI])

    composed = camera_to_target @ grasp @ tool

    # The tool offset is expressed in the grasp frame, so it lands on +Z.
    np.testing.assert_allclose(
        composed[:3, 3],
        (camera_to_target @ np.array([0.1, 0.2, 0.3 + 0.1034, 1.0]))[:3],
    )
    np.testing.assert_allclose(
        composed[:3, :3],
        rotation_matrix_from_rpy(0.0, 0.0, np.pi),
        atol=1e-12,
    )
