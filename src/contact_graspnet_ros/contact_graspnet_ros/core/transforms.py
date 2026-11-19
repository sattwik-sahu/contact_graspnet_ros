"""Small homogeneous-transform helpers shared by the core, ROS layer and tests.

Everything here is pure NumPy so the module can be imported without ROS. The
conventions match ROS / REP 103:

* rotations are ``4x4`` homogeneous matrices,
* roll/pitch/yaw are fixed-axis ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``,
* quaternions are ``(x, y, z, w)``.
"""

from __future__ import annotations

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float64]
Matrix4 = npt.NDArray[np.float64]


def rotation_matrix_from_rpy(roll: float, pitch: float, yaw: float) -> FloatArray:
    """Return the ``3x3`` rotation for fixed-axis roll/pitch/yaw (radians)."""

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def rotation_matrix_from_quaternion(
    quaternion: npt.ArrayLike,
) -> FloatArray:
    """Return the ``3x3`` rotation for an ``(x, y, z, w)`` quaternion."""

    x, y, z, w = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = np.linalg.norm([x, y, z, w])
    if norm < 1e-12:
        raise ValueError("Quaternion has zero norm")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_from_xyz_quaternion(
    translation: npt.ArrayLike,
    quaternion: npt.ArrayLike,
) -> Matrix4:
    """Build a ``4x4`` transform from a translation and ``(x, y, z, w)`` quaternion."""

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix_from_quaternion(quaternion)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def transform_from_offset_rpy(
    offset_xyz: npt.ArrayLike,
    rpy: npt.ArrayLike,
) -> Matrix4:
    """Build a ``4x4`` transform from an xyz offset and roll/pitch/yaw."""

    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix_from_rpy(*np.asarray(rpy, dtype=np.float64))
    transform[:3, 3] = np.asarray(offset_xyz, dtype=np.float64).reshape(3)
    return transform


def apply_transform(transform: npt.ArrayLike, points: npt.ArrayLike) -> FloatArray:
    """Apply a ``4x4`` transform to one point or an ``(N, 3)`` array of points."""

    matrix = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    array = np.asarray(points, dtype=np.float64)
    single = array.ndim == 1
    if single:
        array = array.reshape(1, 3)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError(f"points must be (3,) or (N, 3), got {array.shape}")

    transformed = array @ matrix[:3, :3].T + matrix[:3, 3]
    return transformed[0] if single else transformed
