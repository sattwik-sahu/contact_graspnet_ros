"""ROS-free geometry and grasp-selection helpers.

The functions in this module are a typed, headless adaptation of the pure
NumPy helpers in the upstream Contact-GraspNet PyTorch port.  The port's
``data.py`` imports pyrender for training-time rendering, which is undesirable
for a ROS inference node.  Keeping the inference helpers here lets the node run
without a rendering stack while leaving the submodule untouched.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping

import numpy as np

from .types import FloatArray, IntArray

_EPS = 1e-12


def depth_to_point_cloud(depth: FloatArray, K: FloatArray) -> FloatArray:
    """Convert a metric depth image to an ``(N, 3)`` point cloud.

    Args:
        depth: ``(H, W)`` depth image in metres.  Non-positive values are
            treated as invalid and skipped.
        K: ``(3, 3)`` pinhole camera matrix.  Only ``fx``, ``fy``, ``cx`` and
            ``cy`` are used.

    Returns:
        ``(N, 3)`` float32 point cloud in the camera optical frame.
    """

    depth = np.asarray(depth, dtype=np.float32)
    K = np.asarray(K, dtype=np.float32).reshape(3, 3)
    if depth.ndim != 2:
        raise ValueError(f"depth must be 2D, got shape {depth.shape}")

    mask = np.where(depth > 0)
    y, x = mask[0], mask[1]
    if x.size == 0:
        return np.empty((0, 3), dtype=np.float32)

    z = depth[y, x]
    world_x = (x.astype(np.float32) - K[0, 2]) * z / K[0, 0]
    world_y = (y.astype(np.float32) - K[1, 2]) * z / K[1, 1]
    return np.stack((world_x, world_y, z), axis=1).astype(np.float32)


def reject_median_outliers(
    data: FloatArray, m: float = 0.4, z_only: bool = False
) -> FloatArray:
    """Reject points whose absolute deviation from the median exceeds ``m``."""

    data = np.asarray(data, dtype=np.float32)
    if data.size == 0:
        return data
    if z_only:
        deviation = np.abs(data[:, 2:3] - np.median(data[:, 2:3]))
    else:
        deviation = np.abs(data - np.median(data, axis=0, keepdims=True))
    return data[np.sum(deviation, axis=1) < m]


def farthest_points(
    data: FloatArray,
    nclusters: int,
    dist_func,
    return_center_indexes: bool = False,
    return_distances: bool = False,
    verbose: bool = False,
):
    """Perform farthest point sampling on ``data``.

    This mirrors the upstream helper and is used to select a spatially diverse
    subset of grasp contacts.
    """

    data = np.asarray(data, dtype=np.float32)
    if nclusters >= data.shape[0]:
        indexes = np.arange(data.shape[0], dtype=np.int32)
        if return_center_indexes:
            return indexes, indexes
        return indexes

    clusters = np.full((data.shape[0],), -1, dtype=np.int32)
    distances = np.full((data.shape[0],), 1e7, dtype=np.float32)
    centers: list[int] = []
    for iteration in range(nclusters):
        index = int(np.argmax(distances))
        centers.append(index)
        broadcasted = np.broadcast_to(data[index], data.shape)
        new_distances = dist_func(broadcasted, data)
        distances = np.minimum(distances, new_distances)
        clusters[distances == new_distances] = iteration
        if verbose:
            print(f"farthest points max distance: {np.max(distances)}")

    center_array = np.asarray(centers, dtype=np.int32)
    if return_center_indexes:
        if return_distances:
            return clusters, center_array, distances
        return clusters, center_array
    return clusters


def distance_by_translation_point(p1: FloatArray, p2: FloatArray) -> FloatArray:
    """Euclidean distance between corresponding points."""
    return np.sqrt(np.sum(np.square(p1 - p2), axis=-1))


def regularize_pc_point_count(
    pc: FloatArray,
    npoints: int,
    rng: np.random.Generator | None = None,
    use_farthest_point: bool = False,
) -> FloatArray:
    """Downsample or oversample a point cloud to exactly ``npoints`` points."""

    pc = np.asarray(pc, dtype=np.float32)
    npoints = int(npoints)
    if npoints <= 0:
        raise ValueError(f"npoints must be positive, got {npoints}")
    if pc.shape[0] == 0:
        raise ValueError("cannot regularize an empty point cloud")

    generator = rng if rng is not None else np.random.default_rng()
    if pc.shape[0] > npoints:
        if use_farthest_point:
            _, centers = farthest_points(
                pc, npoints, distance_by_translation_point, return_center_indexes=True
            )
            indexes = centers
        else:
            indexes = generator.choice(pc.shape[0], size=npoints, replace=False)
        return pc[indexes, :]

    required = npoints - pc.shape[0]
    if required > 0:
        indexes = generator.choice(pc.shape[0], size=required, replace=True)
        pc = np.concatenate((pc, pc[indexes, :]), axis=0)
    return pc


def preprocess_pc_for_inference(
    input_pc: FloatArray,
    num_point: int,
    rng: np.random.Generator | None = None,
    return_mean: bool = True,
    use_farthest_point: bool = False,
    convert_to_internal_coords: bool = True,
) -> tuple[FloatArray, FloatArray] | FloatArray:
    """Regularize, center and optionally flip a point cloud for the network.

    The network was trained in an internal camera frame with x left, y up and z
    forward.  The public ROS interface uses the OpenCV/REP 103 optical frame, so
    x and y are flipped here and flipped back after inference.
    """

    pc = np.asarray(input_pc, dtype=np.float32)
    if pc.ndim != 2 or pc.shape[1] != 3:
        raise ValueError(f"point cloud must be (N, 3), got {pc.shape}")

    if pc.shape[0] != num_point:
        pc = regularize_pc_point_count(
            pc, num_point, rng=rng, use_farthest_point=use_farthest_point
        ).copy()
    else:
        pc = pc.copy()

    if convert_to_internal_coords:
        pc[:, :2] *= -1.0

    mean = np.mean(pc, axis=0).astype(np.float32)
    pc -= mean
    if return_mean:
        return pc, mean
    return pc


def extract_point_clouds(
    depth: FloatArray,
    K: FloatArray,
    mask: IntArray | None,
    *,
    z_range: tuple[float, float] = (0.2, 1.8),
    segmap_id: int = 0,
    skip_border_objects: bool = False,
    margin_px: int = 5,
) -> tuple[FloatArray, dict[int, FloatArray]]:
    """Convert depth to a full point cloud and optional per-label segments.

    Args:
        depth: ``(H, W)`` metric depth image.
        K: ``(3, 3)`` camera matrix.
        mask: Optional ``(H, W)`` integer label image.  ``0`` is background.
        z_range: Minimum/maximum camera z in metres.
        segmap_id: ``0`` keeps all positive labels; a positive value keeps only
            that label.
        skip_border_objects: Skip segments touching the image border.
        margin_px: Border margin in pixels for ``skip_border_objects``.

    Returns:
        ``(pc_full, pc_segments)`` where ``pc_segments`` maps label id to an
        ``(N, 3)`` point cloud.
    """

    depth = np.asarray(depth, dtype=np.float32)
    K = np.asarray(K, dtype=np.float32).reshape(3, 3)
    pc_full = depth_to_point_cloud(depth, K)
    z_mask = (pc_full[:, 2] < float(z_range[1])) & (pc_full[:, 2] > float(z_range[0]))
    pc_full = pc_full[z_mask]

    pc_segments: dict[int, FloatArray] = {}
    if mask is None:
        return pc_full, pc_segments

    mask = np.asarray(mask, dtype=np.int32)
    if mask.shape != depth.shape:
        raise ValueError(
            f"mask shape {mask.shape} does not match depth shape {depth.shape}"
        )

    if int(segmap_id) > 0:
        labels: Iterable[int] = [int(segmap_id)]
    else:
        labels = np.unique(mask[mask > 0]).tolist()

    height, width = depth.shape
    for label in labels:
        label = int(label)
        if label <= 0:
            continue
        if skip_border_objects and label != int(segmap_id):
            obj_y, obj_x = np.where(mask == label)
            if (
                np.any(obj_x < margin_px)
                or np.any(obj_x >= width - margin_px)
                or np.any(obj_y < margin_px)
                or np.any(obj_y >= height - margin_px)
            ):
                continue
        instance_mask = mask == label
        if not np.any(instance_mask):
            continue
        segment = depth_to_point_cloud(depth * instance_mask.astype(np.float32), K)
        segment = segment[
            (segment[:, 2] < float(z_range[1])) & (segment[:, 2] > float(z_range[0]))
        ]
        if segment.shape[0] > 0:
            pc_segments[label] = segment

    return pc_full, pc_segments


def extract_3d_cam_boxes(
    full_pc: FloatArray,
    pc_segments: Mapping[int, FloatArray],
    raw_num_points: int,
    rng: np.random.Generator,
    *,
    min_size: float = 0.2,
    max_size: float = 0.6,
) -> tuple[dict[int, FloatArray], dict[int, FloatArray]]:
    """Crop a context cube around each segment and regularize its point count."""

    pc_regions: dict[int, FloatArray] = {}
    obj_centers: dict[int, FloatArray] = {}

    for key, segment in pc_segments.items():
        segment = reject_median_outliers(np.asarray(segment, dtype=np.float32), m=0.4)
        if segment.size == 0:
            continue

        max_bounds = np.max(segment[:, :3], axis=0)
        min_bounds = np.min(segment[:, :3], axis=0)
        extent = max_bounds - min_bounds
        center = min_bounds + extent / 2.0
        size = float(np.minimum(np.maximum(np.max(extent) * 2.0, min_size), max_size))

        partial = full_pc[
            np.all(full_pc > (center - size / 2.0), axis=1)
            & np.all(full_pc < (center + size / 2.0), axis=1)
        ]
        if partial.shape[0] == 0:
            continue
        pc_regions[int(key)] = regularize_pc_point_count(
            partial, raw_num_points, rng=rng
        )
        obj_centers[int(key)] = center.astype(np.float32)

    return pc_regions, obj_centers


def select_grasps(
    contact_pts: FloatArray,
    contact_conf: FloatArray,
    max_farthest_points: int = 150,
    num_grasps: int = 200,
    first_thres: float = 0.25,
    second_thres: float = 0.2,
    with_replacement: bool = False,
) -> IntArray:
    """Select a diverse, high-confidence subset of grasp contacts."""

    contact_pts = np.asarray(contact_pts, dtype=np.float32).reshape(-1, 3)
    contact_conf = np.asarray(contact_conf, dtype=np.float32).reshape(-1)
    if contact_pts.shape[0] != contact_conf.shape[0]:
        raise ValueError("contact_pts and contact_conf must have the same length")

    conf_indexes = np.nonzero(contact_conf > first_thres)[0]
    if conf_indexes.size == 0:
        return np.empty((0,), dtype=np.int32)

    _, center_indexes = farthest_points(
        contact_pts[conf_indexes, :3],
        int(min(max_farthest_points, len(conf_indexes))),
        distance_by_translation_point,
        return_center_indexes=True,
    )
    selected_from_first = conf_indexes[center_indexes]

    remaining = np.setdiff1d(np.arange(len(contact_conf)), selected_from_first)
    sorted_confidence = np.argsort(contact_conf)[::-1]
    mask = np.isin(sorted_confidence, remaining)
    sorted_remaining = sorted_confidence[mask]

    if with_replacement:
        selection = list(selected_from_first)
        j = len(selection)
        while j < num_grasps and conf_indexes.size > 0:
            selection.append(conf_indexes[j % conf_indexes.size])
            j += 1
        return np.asarray(selection, dtype=np.int32)

    remaining_idcs = sorted_remaining[: max(0, num_grasps - len(selected_from_first))]
    remaining_above_second = np.nonzero(contact_conf[remaining_idcs] > second_thres)[0]
    return np.union1d(
        selected_from_first,
        remaining_idcs[remaining_above_second],
    ).astype(np.int32)


def filter_segment(
    contact_pts: FloatArray, segment_pc: FloatArray, thres: float = 0.0001
) -> IntArray:
    """Return indices of contacts lying within ``thres`` of a segment cloud."""

    contact_pts = np.asarray(contact_pts, dtype=np.float32).reshape(-1, 3)
    segment_pc = np.asarray(segment_pc, dtype=np.float32).reshape(-1, 3)
    if contact_pts.shape[0] == 0 or segment_pc.shape[0] == 0:
        return np.empty((0,), dtype=np.int32)

    distances = contact_pts[:, None, :] - segment_pc[None, :, :]
    min_distances = np.min(np.linalg.norm(distances, axis=2), axis=1)
    return np.nonzero(min_distances < thres)[0].astype(np.int32)


def build_6d_grasp_numpy(
    approach_dirs: FloatArray,
    base_dirs: FloatArray,
    contact_pts: FloatArray,
    thickness: FloatArray,
    gripper_depth: float = 0.1034,
) -> FloatArray:
    """Build 4x4 gripper-base poses from directions, contacts and widths.

    This NumPy implementation is used by unit tests and documents the pose
    convention; the network uses the equivalent Torch path in ``model.py``.
    """

    approach_dirs = np.asarray(approach_dirs, dtype=np.float32)
    base_dirs = np.asarray(base_dirs, dtype=np.float32)
    contact_pts = np.asarray(contact_pts, dtype=np.float32)
    thickness = np.asarray(thickness, dtype=np.float32).reshape(-1)

    grasps = np.tile(np.eye(4, dtype=np.float32), (contact_pts.shape[0], 1, 1))
    for i in range(contact_pts.shape[0]):
        base = base_dirs[i] / max(np.linalg.norm(base_dirs[i]), _EPS)
        approach = approach_dirs[i] / max(np.linalg.norm(approach_dirs[i]), _EPS)
        grasp_y = np.cross(approach, base)
        grasp_y /= max(np.linalg.norm(grasp_y), _EPS)
        grasps[i, :3, 0] = base
        grasps[i, :3, 1] = grasp_y
        grasps[i, :3, 2] = approach
        grasps[i, :3, 3] = (
            contact_pts[i] + thickness[i] / 2.0 * base - gripper_depth * approach
        )
    return grasps


__all__ = [
    "build_6d_grasp_numpy",
    "depth_to_point_cloud",
    "distance_by_translation_point",
    "extract_3d_cam_boxes",
    "extract_point_clouds",
    "farthest_points",
    "filter_segment",
    "preprocess_pc_for_inference",
    "regularize_pc_point_count",
    "reject_median_outliers",
    "select_grasps",
]
