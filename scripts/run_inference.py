#!/usr/bin/env python3
"""Headless Contact-GraspNet inference helper.

Example:
    pixi run -e jazzy python scripts/run_inference.py \
        --input third_party/contact_graspnet_pytorch/test_data/7.npy \
        --output /tmp/predictions.npz

This script does not import ROS.  It is useful for validating the engine and the
checkpoint before launching the node.
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "contact_graspnet_ros"))

from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.types import (
    CameraIntrinsics,
    InferenceOptions,
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help=".npz/.npy input")
    parser.add_argument(
        "--config", type=Path, default=REPO_ROOT / "config" / "model.yaml"
    )
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=REPO_ROOT
        / "third_party"
        / "contact_graspnet_pytorch"
        / "checkpoints"
        / "contact_graspnet"
        / "checkpoints"
        / "model.pt",
    )
    parser.add_argument("--device", default="cuda", choices=("cuda", "cpu", "auto"))
    parser.add_argument(
        "--output", type=Path, default=Path("/tmp/contact_graspnet_predictions.npz")
    )
    parser.add_argument("--segmap-id", type=int, default=0)
    parser.add_argument("--z-min", type=float, default=0.2)
    parser.add_argument("--z-max", type=float, default=1.8)
    parser.add_argument("--forward-passes", type=int, default=1)
    parser.add_argument("--first-thres", type=float, default=0.15)
    parser.add_argument("--second-thres", type=float, default=0.15)
    return parser.parse_args()


def _load_input(path: Path):
    data = np.load(path, allow_pickle=True)
    if hasattr(data, "files"):
        keys = data.files
        depth = np.asarray(data["depth"], dtype=np.float32) if "depth" in keys else None
        K = (
            np.asarray(data["K"], dtype=np.float32).reshape(3, 3)
            if "K" in keys
            else None
        )
        mask = np.asarray(data["segmap"], dtype=np.int32) if "segmap" in keys else None
        points = np.asarray(data["xyz"], dtype=np.float32) if "xyz" in keys else None
        return depth, K, mask, points
    array = np.asarray(data)
    if array.ndim == 0:
        item = array.item()
        if isinstance(item, dict):
            keys = item.keys()
            depth = (
                np.asarray(item["depth"], dtype=np.float32) if "depth" in keys else None
            )
            K = (
                np.asarray(item["K"], dtype=np.float32).reshape(3, 3)
                if "K" in keys
                else None
            )
            mask_key = (
                "segmap" if "segmap" in keys else "seg" if "seg" in keys else None
            )
            mask = np.asarray(item[mask_key], dtype=np.int32) if mask_key else None
            points = (
                np.asarray(item["xyz"], dtype=np.float32) if "xyz" in keys else None
            )
            return depth, K, mask, points
        raise ValueError(f"Unsupported 0-d input of type {type(item)}")
    if array.ndim == 2 and array.shape[1] == 3:
        return None, None, None, array.astype(np.float32)
    if array.ndim == 2:
        return array.astype(np.float32), None, None, None
    raise ValueError(f"Unsupported input shape {array.shape}")


def main() -> None:
    args = _parse_args()
    depth, K, mask, points = _load_input(args.input)

    engine = ContactGraspNetEngine(
        model_config_path=args.config,
        checkpoint_path=args.checkpoint,
        device=args.device,
        seed=0,
    )
    options = InferenceOptions(
        z_range=(args.z_min, args.z_max),
        segmap_id=args.segmap_id,
        forward_passes=args.forward_passes,
        first_thres=args.first_thres,
        second_thres=args.second_thres,
    )

    try:
        if points is not None:
            if mask is not None:
                segment = points[mask.reshape(-1) > 0]
                options = replace(
                    options,
                    segmap_id=args.segmap_id if args.segmap_id > 0 else 1,
                )
                result = engine.infer_pointclouds(points, segment, options)
            else:
                options = replace(options, local_regions=False, filter_grasps=False)
                result = engine.infer_pointclouds(points, None, options)
        else:
            if depth is None or K is None:
                raise ValueError("Depth input requires both 'depth' and 'K'")
            if mask is not None and mask.shape != depth.shape:
                raise ValueError(
                    f"mask shape {mask.shape} != depth shape {depth.shape}"
                )
            if mask is None:
                options = replace(options, local_regions=False, filter_grasps=False)
            intrinsics = CameraIntrinsics.from_matrix(
                K, width=depth.shape[1], height=depth.shape[0]
            )
            result = engine.infer(depth, intrinsics, mask, options)
    finally:
        engine.close()

    poses = np.asarray([p.pose for p in result.proposals], dtype=np.float32).reshape(
        -1, 4, 4
    )
    scores = np.asarray([p.score for p in result.proposals], dtype=np.float32)
    widths = np.asarray([p.width for p in result.proposals], dtype=np.float32)
    contacts = np.asarray(
        [p.contact_point for p in result.proposals], dtype=np.float32
    ).reshape(-1, 3)
    segment_ids = np.asarray([p.segment_id for p in result.proposals], dtype=np.int32)

    np.savez(
        args.output,
        poses=poses,
        scores=scores,
        widths=widths,
        contacts=contacts,
        segment_ids=segment_ids,
    )
    print(
        f"{len(result)} grasps | {result.num_input_points} input points | "
        f"{result.inference_time_s:.3f}s | device={engine.device_name}"
    )
    print(f"Saved to {args.output}")
    for warning in result.warnings:
        print(f"warning: {warning}")


if __name__ == "__main__":
    main()
