"""ROS-free Contact-GraspNet inference engine."""

from __future__ import annotations

import contextlib
import time
from collections.abc import Mapping
from pathlib import Path

import numpy as np

from .config import load_model_config, resolve_model_defaults
from .geometry import (
    extract_3d_cam_boxes,
    extract_point_clouds,
    filter_segment,
    preprocess_pc_for_inference,
    regularize_pc_point_count,
    select_grasps,
)
from .model import build_model, load_checkpoint, resolve_device, set_seed
from .types import (
    CameraIntrinsics,
    FloatArray,
    GraspProposal,
    GraspResult,
    InferenceOptions,
    IntArray,
    ModelDefaults,
)


class ContactGraspNetEngine:
    """Load the PyTorch Contact-GraspNet model and run depth-based inference.

    The engine is intentionally unaware of ROS.  It accepts NumPy arrays and
    returns typed dataclasses so it can be tested without sourcing a ROS
    environment.
    """

    def __init__(
        self,
        model_config_path: str | Path,
        checkpoint_path: str | Path,
        device: str = "cuda",
        seed: int = 0,
    ) -> None:
        self._config = load_model_config(model_config_path)
        self._defaults = resolve_model_defaults(self._config)
        self._device = resolve_device(device)
        set_seed(seed)
        self._rng = np.random.default_rng(int(seed))

        self._model = build_model(self._config, self._device)
        load_checkpoint(self._model, checkpoint_path, self._device)
        self._torch = __import__("torch")

    @property
    def defaults(self) -> ModelDefaults:
        return self._defaults

    @property
    def device_name(self) -> str:
        return str(self._device)

    def infer(
        self,
        depth: FloatArray,
        intrinsics: CameraIntrinsics,
        mask: IntArray | None,
        options: InferenceOptions,
    ) -> GraspResult:
        """Run inference from a metric depth image and optional label mask."""

        options.validate()
        started = time.perf_counter()

        depth_array = np.asarray(depth, dtype=np.float32)
        if depth_array.ndim != 2:
            raise ValueError(f"depth must be (H, W), got {depth_array.shape}")
        if depth_array.shape != (intrinsics.height, intrinsics.width):
            raise ValueError(
                "depth shape does not match camera intrinsics: "
                f"depth={depth_array.shape}, camera_info=({intrinsics.height}, {intrinsics.width})"
            )

        mask_array: IntArray | None = None
        if mask is not None:
            mask_array = np.asarray(mask, dtype=np.int32)
            if mask_array.shape != depth_array.shape:
                raise ValueError(
                    f"mask shape {mask_array.shape} does not match depth shape {depth_array.shape}"
                )

        if (options.local_regions or options.filter_grasps) and mask_array is None:
            raise ValueError("local_regions/filter_grasps require a segmentation mask")

        pc_full, pc_segments = extract_point_clouds(
            depth_array,
            intrinsics.matrix(),
            mask_array,
            z_range=options.z_range,
            segmap_id=options.segmap_id,
            skip_border_objects=options.skip_border_objects,
        )
        if pc_full.shape[0] == 0:
            return GraspResult(
                proposals=(),
                num_input_points=0,
                inference_time_s=float(time.perf_counter() - started),
                warnings=("empty point cloud after depth/z_range filtering",),
            )
        result = self._predict_scene(pc_full, pc_segments, options, started)
        return result

    def infer_pointclouds(
        self,
        points: FloatArray,
        segment: FloatArray | None,
        options: InferenceOptions,
    ) -> GraspResult:
        """Run inference from precomputed point clouds in the camera frame."""

        options.validate()
        started = time.perf_counter()

        points_array = np.asarray(points, dtype=np.float32)
        if points_array.ndim != 2 or points_array.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got {points_array.shape}")
        if points_array.shape[0] == 0:
            raise ValueError("points must not be empty")

        pc_segments: dict[int, FloatArray] = {}
        if segment is not None:
            segment_array = np.asarray(segment, dtype=np.float32)
            if segment_array.ndim != 2 or segment_array.shape[1] != 3:
                raise ValueError(f"segment must be (N, 3), got {segment_array.shape}")
            if segment_array.shape[0] == 0:
                raise ValueError("segment must not be empty")
            if options.segmap_id <= 0:
                raise ValueError(
                    "segmap_id must be > 0 when a segment cloud is supplied"
                )
            pc_segments[int(options.segmap_id)] = segment_array

        if (options.local_regions or options.filter_grasps) and not pc_segments:
            raise ValueError("local_regions/filter_grasps require a segment cloud")

        return self._predict_scene(points_array, pc_segments, options, started)

    def close(self) -> None:
        """Release the model and cached CUDA memory."""

        self._model = None
        if str(self._device).startswith("cuda"):
            with contextlib.suppress(Exception):
                self._torch.cuda.empty_cache()

    def _predict_scene(
        self,
        pc_full: FloatArray,
        pc_segments: Mapping[int, FloatArray],
        options: InferenceOptions,
        started: float,
    ) -> GraspResult:
        warnings: list[str] = []
        pred_grasps: dict[int, FloatArray] = {}
        pred_scores: dict[int, FloatArray] = {}
        pred_contacts: dict[int, FloatArray] = {}
        pred_widths: dict[int, FloatArray] = {}

        if options.local_regions:
            if len(pc_segments) > options.max_segments:
                warnings.append(
                    f"mask contains {len(pc_segments)} segments; considering the first "
                    f"{options.max_segments} (max_segments)"
                )
                pc_segments = dict(sorted(pc_segments.items())[: options.max_segments])

            regions, _ = extract_3d_cam_boxes(
                pc_full,
                pc_segments,
                self._defaults.raw_num_points,
                self._rng,
                min_size=0.2,
                max_size=0.6,
            )
            keys = sorted(regions.keys())
            if len(keys) > options.max_segments:
                warnings.append(
                    f"local_regions produced {len(keys)} segments; processing the first "
                    f"{options.max_segments} (max_segments)"
                )
                keys = keys[: options.max_segments]

            for key in keys:
                grasps, scores, contacts, widths = self._predict_grasps(
                    regions[key], options
                )
                pred_grasps[key] = grasps
                pred_scores[key] = scores
                pred_contacts[key] = contacts
                pred_widths[key] = widths
        else:
            pc_full = regularize_pc_point_count(
                pc_full,
                self._defaults.raw_num_points,
                rng=self._rng,
                use_farthest_point=self._defaults.use_farthest_point,
            )
            grasps, scores, contacts, widths = self._predict_grasps(pc_full, options)
            pred_grasps[-1] = grasps
            pred_scores[-1] = scores
            pred_contacts[-1] = contacts
            pred_widths[-1] = widths

        if options.filter_grasps:
            if not options.local_regions and len(pc_segments) > options.max_segments:
                warnings.append(
                    f"mask contains {len(pc_segments)} segments; filtering the first "
                    f"{options.max_segments} (max_segments)"
                )
                pc_segments = dict(sorted(pc_segments.items())[: options.max_segments])

            segment_keys = (
                list(pred_grasps.keys())
                if options.local_regions
                else list(pc_segments.keys())
            )
            for key in segment_keys:
                if key not in pc_segments:
                    continue
                source_key = key if options.local_regions else -1
                if source_key not in pred_contacts:
                    continue
                if (
                    pc_segments[key].shape[0] == 0
                    or pred_contacts[source_key].shape[0] == 0
                ):
                    continue
                indexes = filter_segment(
                    pred_contacts[source_key],
                    pc_segments[key],
                    thres=self._defaults.filter_thres,
                )
                pred_grasps[key] = pred_grasps[source_key][indexes]
                pred_scores[key] = pred_scores[source_key][indexes]
                pred_contacts[key] = pred_contacts[source_key][indexes]
                pred_widths[key] = pred_widths[source_key][indexes]

            if not options.local_regions and any(key != -1 for key in pred_grasps):
                pred_grasps.pop(-1, None)
                pred_scores.pop(-1, None)
                pred_contacts.pop(-1, None)
                pred_widths.pop(-1, None)

        proposals = _flatten_proposals(
            pred_grasps, pred_scores, pred_contacts, pred_widths
        )
        return GraspResult(
            proposals=tuple(proposals),
            num_input_points=int(pc_full.shape[0]),
            inference_time_s=float(time.perf_counter() - started),
            warnings=tuple(warnings),
        )

    def _predict_grasps(
        self, pc: FloatArray, options: InferenceOptions
    ) -> tuple[FloatArray, FloatArray, FloatArray, FloatArray]:
        torch = self._torch
        pc_pre, pc_mean = preprocess_pc_for_inference(
            pc,
            self._defaults.raw_num_points,
            rng=self._rng,
            return_mean=True,
            use_farthest_point=self._defaults.use_farthest_point,
            convert_to_internal_coords=True,
        )
        pc_batch = np.asarray(pc_pre, dtype=np.float32)[None, :, :]
        if options.forward_passes > 1:
            pc_batch = np.tile(pc_batch, (int(options.forward_passes), 1, 1))

        if str(self._device).startswith("cuda"):
            torch.cuda.synchronize(self._device)

        tensor = torch.from_numpy(np.ascontiguousarray(pc_batch)).to(self._device)
        with torch.inference_mode():
            prediction = self._model(tensor)

        grasps = prediction["pred_grasps_cam"].detach().cpu().numpy().reshape(-1, 4, 4)
        scores = prediction["pred_scores"].detach().cpu().numpy().reshape(-1)
        points = prediction["pred_points"].detach().cpu().numpy().reshape(-1, 3)
        offsets = prediction["offset_pred"].detach().cpu().numpy().reshape(-1)

        if str(self._device).startswith("cuda"):
            torch.cuda.synchronize(self._device)

        mean = np.asarray(pc_mean, dtype=np.float32).reshape(1, 3)
        grasps[:, :3, 3] += mean
        points[:, :3] += mean

        widths = np.minimum(
            offsets + self._defaults.extra_opening,
            self._defaults.gripper_width,
        ).astype(np.float32)

        selection = select_grasps(
            points[:, :3],
            scores,
            self._defaults.max_farthest_points,
            self._defaults.num_samples,
            options.first_thres,
            options.second_thres,
            with_replacement=bool(
                self._config.get("TEST", {}).get("with_replacement", False)
            ),
        )
        if selection.size == 0:
            empty_grasps = np.empty((0, 4, 4), dtype=np.float32)
            empty_values = np.empty((0,), dtype=np.float32)
            empty_points = np.empty((0, 3), dtype=np.float32)
            return empty_grasps, empty_values, empty_points, empty_values

        grasps = grasps[selection].astype(np.float32)
        scores = scores[selection].astype(np.float32)
        points = points[selection].astype(np.float32)
        widths = widths[selection].astype(np.float32)

        if self._defaults.center_to_tip:
            grasps[:, :3, 3] -= grasps[:, :3, 2] * (self._defaults.center_to_tip / 2.0)

        # Convert from the internal x-left/y-up frame back to the OpenCV/ROS
        # optical frame used by the public interface.
        grasps[:, :2, :] *= -1.0
        points[:, :2] *= -1.0

        return grasps, scores, points, widths


def _flatten_proposals(
    pred_grasps: Mapping[int, FloatArray],
    pred_scores: Mapping[int, FloatArray],
    pred_contacts: Mapping[int, FloatArray],
    pred_widths: Mapping[int, FloatArray],
) -> list[GraspProposal]:
    keys = sorted(pred_grasps.keys())
    if len(keys) > 1 and -1 in keys:
        keys.remove(-1)

    proposals: list[GraspProposal] = []
    for key in keys:
        grasps = np.asarray(pred_grasps.get(key, []), dtype=np.float32).reshape(
            -1, 4, 4
        )
        scores = np.asarray(pred_scores.get(key, []), dtype=np.float32).reshape(-1)
        contacts = np.asarray(pred_contacts.get(key, []), dtype=np.float32).reshape(
            -1, 3
        )
        widths = np.asarray(pred_widths.get(key, []), dtype=np.float32).reshape(-1)
        count = min(
            grasps.shape[0], scores.shape[0], contacts.shape[0], widths.shape[0]
        )
        for index in range(count):
            proposals.append(
                GraspProposal(
                    pose=grasps[index].astype(np.float32),
                    score=float(scores[index]),
                    width=float(widths[index]),
                    contact_point=contacts[index].astype(np.float32),
                    segment_id=int(key),
                )
            )

    proposals.sort(key=lambda proposal: proposal.score, reverse=True)
    return proposals


__all__ = ["ContactGraspNetEngine"]
