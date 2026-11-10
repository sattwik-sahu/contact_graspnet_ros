"""Typed data structures shared by the ROS node and the ROS-free engine."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
import numpy.typing as npt

FloatArray = npt.NDArray[np.float32]
IntArray = npt.NDArray[np.int32]
BoolArray = npt.NDArray[np.bool_]


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics for a rectified depth image.

    The model uses only ``fx``, ``fy``, ``cx`` and ``cy``.  The optical frame is
    assumed to follow REP 103: x right, y down, z forward.
    """

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def matrix(self) -> FloatArray:
        """Return the intrinsic matrix as a row-major ``(3, 3)`` array."""
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )

    @classmethod
    def from_matrix(
        cls, matrix: npt.ArrayLike, width: int, height: int
    ) -> CameraIntrinsics:
        """Build intrinsics from a 3x3 camera matrix."""
        k = np.asarray(matrix, dtype=np.float32).reshape(3, 3)
        return cls(
            fx=float(k[0, 0]),
            fy=float(k[1, 1]),
            cx=float(k[0, 2]),
            cy=float(k[1, 2]),
            width=int(width),
            height=int(height),
        )


@dataclass(frozen=True)
class InferenceOptions:
    """Per-request inference options.

    Defaults mirror the PyTorch port's checkpoint configuration.  The ROS node
    may override these from parameters or service requests.
    """

    z_range: tuple[float, float] = (0.2, 1.8)
    local_regions: bool = True
    filter_grasps: bool = True
    skip_border_objects: bool = False
    segmap_id: int = 0
    max_segments: int = 4
    forward_passes: int = 1
    first_thres: float = 0.15
    second_thres: float = 0.15
    num_samples: int = 200
    max_farthest_points: int = 150

    def validate(self) -> None:
        if len(self.z_range) != 2 or self.z_range[0] >= self.z_range[1]:
            raise ValueError(
                f"z_range must be [min, max] with min < max, got {self.z_range!r}"
            )
        if self.segmap_id < 0:
            raise ValueError(f"segmap_id must be >= 0, got {self.segmap_id}")
        if self.max_segments < 1:
            raise ValueError(f"max_segments must be >= 1, got {self.max_segments}")
        if self.forward_passes < 1:
            raise ValueError(f"forward_passes must be >= 1, got {self.forward_passes}")
        if not (0.0 <= self.first_thres <= 1.0):
            raise ValueError(f"first_thres must be in [0, 1], got {self.first_thres}")
        if not (0.0 <= self.second_thres <= 1.0):
            raise ValueError(f"second_thres must be in [0, 1], got {self.second_thres}")
        if self.num_samples < 1:
            raise ValueError(f"num_samples must be >= 1, got {self.num_samples}")
        if self.max_farthest_points < 1:
            raise ValueError(
                f"max_farthest_points must be >= 1, got {self.max_farthest_points}"
            )


@dataclass(frozen=True)
class GraspProposal:
    """A single 6-DoF grasp proposal in the camera optical frame."""

    pose: FloatArray  # (4, 4), gripper base frame
    score: float
    width: float
    contact_point: FloatArray  # (3,)
    segment_id: int


@dataclass(frozen=True)
class GraspResult:
    """Flattened inference result for one request."""

    proposals: tuple[GraspProposal, ...] = field(default_factory=tuple)
    num_input_points: int = 0
    inference_time_s: float = 0.0
    warnings: tuple[str, ...] = field(default_factory=tuple)

    def __len__(self) -> int:
        return len(self.proposals)


@dataclass(frozen=True)
class ModelDefaults:
    """Architecture and inference defaults read from ``model.yaml``."""

    raw_num_points: int
    num_point: int
    gripper_width: float
    extra_opening: float
    first_thres: float
    second_thres: float
    filter_thres: float
    max_farthest_points: int
    num_samples: int
    use_farthest_point: bool
    center_to_tip: float
    input_normals: bool
