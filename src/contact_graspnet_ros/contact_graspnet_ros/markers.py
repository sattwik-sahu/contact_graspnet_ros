"""RViz marker generation for grasp proposals."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from contact_graspnet_ros.core.types import GraspProposal, GraspResult

_GRIPPER_DEPTH = 0.1034  # metres from gripper base to contact baseline


def build_marker_array(result: GraspResult, frame_id: str, stamp) -> MarkerArray:
    """Build a single LINE_LIST marker containing all grasp hypotheses."""

    marker = Marker()
    marker.header = Header(stamp=stamp, frame_id=frame_id)
    marker.ns = "grasps"
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.002
    marker.pose.orientation.w = 1.0

    for proposal in result.proposals:
        points = _gripper_line_points(proposal)
        color = _score_color(proposal.score)
        for index in range(0, len(points), 2):
            marker.points.append(
                Point(
                    x=float(points[index, 0]),
                    y=float(points[index, 1]),
                    z=float(points[index, 2]),
                )
            )
            marker.points.append(
                Point(
                    x=float(points[index + 1, 0]),
                    y=float(points[index + 1, 1]),
                    z=float(points[index + 1, 2]),
                )
            )
            marker.colors.append(color)
            marker.colors.append(color)

    array = MarkerArray()
    array.markers.append(marker)
    return array


def _gripper_line_points(proposal: GraspProposal) -> np.ndarray:
    """Return world-frame line-segment endpoints for one gripper hypothesis."""

    width = max(float(proposal.width), 0.0)
    half = width / 2.0
    depth = _GRIPPER_DEPTH

    # Local coordinates: +X closing, +Y lateral, +Z approach.  The contact point
    # sits at local x = -width/2 because the pose translation includes the
    # half-opening shift used by the upstream grasp builder.
    local = np.array(
        [
            [-half, 0.0, 0.0],
            [-half, 0.0, depth],
            [half, 0.0, 0.0],
            [half, 0.0, depth],
            [-half, 0.0, 0.0],
            [half, 0.0, 0.0],
            [0.0, 0.0, depth],
            [0.0, 0.0, depth + 0.04],
        ],
        dtype=np.float32,
    )
    pose = np.asarray(proposal.pose, dtype=np.float32).reshape(4, 4)
    world = (pose[:3, :3] @ local.T).T + pose[:3, 3]
    return world.astype(np.float32)


def _score_color(score: float) -> ColorRGBA:
    clipped = float(np.clip(score, 0.0, 1.0))
    return ColorRGBA(r=1.0 - clipped, g=clipped, b=0.0, a=0.85)
