"""Conversions between ROS service messages and the typed NumPy core."""

from __future__ import annotations

import numpy as np
import numpy.typing as npt
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import CameraInfo, Image

from contact_graspnet_ros.core.types import CameraIntrinsics, FloatArray, IntArray

_BRIDGE = CvBridge()


def image_to_depth(message: Image, depth_scale: float = 0.001) -> FloatArray:
    """Convert a ROS depth image to a float32 metric depth array.

    Supports ``32FC1`` (metres) and ``16UC1``/``mono16`` (integer units scaled
    by ``depth_scale``, default millimetres).
    """

    if message.encoding == "32FC1":
        image = _BRIDGE.imgmsg_to_cv2(message, desired_encoding="32FC1")
        depth = np.asarray(image, dtype=np.float32)
    elif message.encoding in ("16UC1", "mono16"):
        image = _BRIDGE.imgmsg_to_cv2(message, desired_encoding="passthrough")
        depth = np.asarray(image, dtype=np.float32) * float(depth_scale)
    else:
        raise ValueError(
            f"Unsupported depth encoding {message.encoding!r}; expected 32FC1 or 16UC1"
        )

    if depth.ndim != 2:
        raise ValueError(f"Depth image must be 2D, got shape {depth.shape}")
    return np.ascontiguousarray(depth, dtype=np.float32)


def image_to_mask(message: Image, binary: bool = False) -> IntArray:
    """Convert a mono8/mono16/32SC1 mask image to an int32 label array."""

    if message.encoding not in ("mono8", "mono16", "32SC1"):
        raise ValueError(
            f"Unsupported mask encoding {message.encoding!r}; expected mono8, mono16 or 32SC1"
        )
    image = _BRIDGE.imgmsg_to_cv2(message, desired_encoding="passthrough")
    mask = np.asarray(image)
    if mask.ndim != 2:
        raise ValueError(f"Mask image must be 2D, got shape {mask.shape}")
    if binary:
        return (mask > 0).astype(np.int32)
    return mask.astype(np.int32, copy=False)


def camera_info_to_intrinsics(message: CameraInfo) -> CameraIntrinsics:
    """Extract rectified intrinsics from a ``CameraInfo`` message.

    The rectified projection matrix ``P`` is preferred. If it is unset, the raw
    intrinsic matrix ``K`` is used.
    """

    if len(message.p) == 12 and message.p[0] > 0.0 and message.p[5] > 0.0:
        fx, fy = float(message.p[0]), float(message.p[5])
        cx, cy = float(message.p[2]), float(message.p[6])
    elif len(message.k) == 9 and message.k[0] > 0.0 and message.k[4] > 0.0:
        fx, fy = float(message.k[0]), float(message.k[4])
        cx, cy = float(message.k[2]), float(message.k[5])
    else:
        raise ValueError("CameraInfo contains no valid P or K intrinsics")

    width = int(message.width)
    height = int(message.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"CameraInfo has invalid size {width}x{height}")
    return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=width, height=height)


def pose_from_matrix(matrix: npt.ArrayLike) -> Pose:
    """Convert a 4x4 gripper-base matrix to a normalized ROS pose."""

    transform = np.asarray(matrix, dtype=np.float64).reshape(4, 4)
    rotation = transform[:3, :3]

    # Re-orthonormalize in case the network output accumulated numerical error.
    u, _, vt = np.linalg.svd(rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt

    quaternion = _rotation_matrix_to_quaternion(rotation)
    return Pose(
        position=Point(
            x=float(transform[0, 3]),
            y=float(transform[1, 3]),
            z=float(transform[2, 3]),
        ),
        orientation=Quaternion(
            x=float(quaternion[0]),
            y=float(quaternion[1]),
            z=float(quaternion[2]),
            w=float(quaternion[3]),
        ),
    )


def _rotation_matrix_to_quaternion(
    rotation: npt.NDArray[np.float64],
) -> npt.NDArray[np.float64]:
    """Convert a 3x3 rotation matrix to an ``[x, y, z, w]`` quaternion."""
    m = rotation
    trace = float(np.trace(m))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        q = np.array(
            [
                (m[2, 1] - m[1, 2]) / s,
                (m[0, 2] - m[2, 0]) / s,
                (m[1, 0] - m[0, 1]) / s,
                0.25 * s,
            ]
        )
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        q = np.array(
            [
                0.25 * s,
                (m[0, 1] + m[1, 0]) / s,
                (m[0, 2] + m[2, 0]) / s,
                (m[2, 1] - m[1, 2]) / s,
            ]
        )
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        q = np.array(
            [
                (m[0, 1] + m[1, 0]) / s,
                0.25 * s,
                (m[1, 2] + m[2, 1]) / s,
                (m[0, 2] - m[2, 0]) / s,
            ]
        )
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        q = np.array(
            [
                (m[0, 2] + m[2, 0]) / s,
                (m[1, 2] + m[2, 1]) / s,
                0.25 * s,
                (m[1, 0] - m[0, 1]) / s,
            ]
        )
    return q / max(np.linalg.norm(q), 1e-12)
