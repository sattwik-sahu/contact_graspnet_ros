#!/usr/bin/env python3
"""Run Contact-GraspNet on a mesh/point cloud downloaded from the internet.

The sampled surface points and resulting proposals are written to an ``.npz``
file that can be rendered with ``scripts/render_banner.py``.

Example:
    pixi run -e jazzy python scripts/demo_from_mesh.py \
        --output /tmp/grasps.npz
"""

from __future__ import annotations

import argparse
import tempfile
import urllib.request
from pathlib import Path

import numpy as np
import trimesh
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.types import InferenceOptions

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_MESH = (
    "https://raw.githubusercontent.com/mrdoob/three.js/dev/examples/"
    "models/ply/binary/Lucy100k.ply"
)
DEFAULT_CONFIG = REPO_ROOT / "config" / "model.yaml"
DEFAULT_CHECKPOINT = (
    REPO_ROOT
    / "third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt"
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh", default=DEFAULT_MESH, help="URL or local .ply/.obj file")
    parser.add_argument("--num-points", type=int, default=20000)
    parser.add_argument("--object-size", type=float, default=0.20, help="metres")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--output", type=Path, default=Path("/tmp/grasps.npz"))
    return parser.parse_args()


def _load_mesh(source: str) -> trimesh.Trimesh:
    if source.startswith(("http://", "https://")):
        suffix = Path(source).suffix or ".ply"
        with tempfile.NamedTemporaryFile(suffix=suffix) as tmp:
            urllib.request.urlretrieve(source, tmp.name)
            return trimesh.load(tmp.name)
    return trimesh.load(source)


def main() -> None:
    args = _parse_args()
    mesh = _load_mesh(args.mesh)
    points = mesh.sample(args.num_points)

    # Centre, scale, and place ~60 cm in front of the camera (optical frame).
    points = points - points.mean(axis=0)
    points = points / np.max(points.max(axis=0) - points.min(axis=0))
    points = (points * args.object_size).astype(np.float32)
    points[:, 2] += 0.60

    engine = ContactGraspNetEngine(
        model_config_path=DEFAULT_CONFIG,
        checkpoint_path=DEFAULT_CHECKPOINT,
        device=args.device,
        seed=0,
    )
    options = InferenceOptions(
        z_range=(0.2, 1.5),
        local_regions=True,
        filter_grasps=True,
        segmap_id=1,
        forward_passes=1,
    )
    result = engine.infer_pointclouds(points, points.copy(), options)
    print(
        f"{len(result)} grasps from {result.num_input_points} points "
        f"in {result.inference_time_s:.3f}s ({args.mesh})"
    )
    for warning in result.warnings:
        print(f"warning: {warning}")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        args.output,
        points=points,
        poses=np.stack([p.pose for p in result.proposals]) if result.proposals else np.empty((0, 4, 4)),
        scores=np.asarray([p.score for p in result.proposals] if result.proposals else [], dtype=np.float32),
        widths=np.asarray([p.width for p in result.proposals] if result.proposals else [], dtype=np.float32),
        contacts=np.stack([p.contact_point for p in result.proposals]) if result.proposals else np.empty((0, 3)),
    )
    print(f"wrote {args.output}")
    engine.close()


if __name__ == "__main__":
    main()
