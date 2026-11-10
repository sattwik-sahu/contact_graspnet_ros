"""ROS parameter parsing and validation for the service-only node."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from rclpy.node import Node

from contact_graspnet_ros.core.types import InferenceOptions


class ConfigError(ValueError):
    """Raised when ROS parameters are invalid."""


@dataclass(frozen=True)
class NodeConfig:
    model_config_path: Path
    checkpoint_dir: Path
    device: str
    z_range: tuple[float, float]
    local_regions: bool
    filter_grasps: bool
    skip_border_objects: bool
    segmap_id: int
    max_segments: int
    forward_passes: int
    first_thres: float
    second_thres: float
    num_samples: int
    max_farthest_points: int
    seed: int
    service_timeout_s: float
    mask_mode: str
    depth_scale: float

    @property
    def checkpoint_path(self) -> Path:
        return self.checkpoint_dir / "model.pt"

    def inference_options(self) -> InferenceOptions:
        return InferenceOptions(
            z_range=self.z_range,
            local_regions=self.local_regions,
            filter_grasps=self.filter_grasps,
            skip_border_objects=self.skip_border_objects,
            segmap_id=self.segmap_id,
            max_segments=self.max_segments,
            forward_passes=self.forward_passes,
            first_thres=self.first_thres,
            second_thres=self.second_thres,
            num_samples=self.num_samples,
            max_farthest_points=self.max_farthest_points,
        )

    @classmethod
    def declare_and_read(cls, node: Node) -> NodeConfig:
        """Declare service parameters with defaults and return a validated config."""

        if not node.has_parameter("use_sim_time"):
            node.declare_parameter("use_sim_time", False)

        model_config = str(node.declare_parameter("model_config", "model.yaml").value)
        checkpoint_dir = str(
            node.declare_parameter(
                "checkpoint_dir",
                "third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints",
            ).value
        )
        device = str(node.declare_parameter("device", "cuda").value)
        z_range_value = node.declare_parameter("z_range", [0.2, 1.8]).value
        local_regions = bool(node.declare_parameter("local_regions", True).value)
        filter_grasps = bool(node.declare_parameter("filter_grasps", True).value)
        skip_border_objects = bool(
            node.declare_parameter("skip_border_objects", False).value
        )
        segmap_id = int(node.declare_parameter("segmap_id", 0).value)
        max_segments = int(node.declare_parameter("max_segments", 4).value)
        forward_passes = int(node.declare_parameter("forward_passes", 1).value)
        first_thres = float(node.declare_parameter("first_thres", 0.15).value)
        second_thres = float(node.declare_parameter("second_thres", 0.15).value)
        num_samples = int(node.declare_parameter("num_samples", 200).value)
        max_farthest_points = int(
            node.declare_parameter("max_farthest_points", 150).value
        )
        seed = int(node.declare_parameter("seed", 0).value)
        service_timeout_s = float(
            node.declare_parameter("service_timeout_s", 5.0).value
        )
        mask_mode = str(node.declare_parameter("mask_mode", "labels").value)
        depth_scale = float(node.declare_parameter("depth_scale", 0.001).value)

        try:
            z_range = (float(z_range_value[0]), float(z_range_value[1]))
        except Exception as exc:
            raise ConfigError(
                f"z_range must be a two-element array, got {z_range_value!r}"
            ) from exc

        config = cls(
            model_config_path=_resolve_model_config(model_config),
            checkpoint_dir=_resolve_checkpoint_dir(checkpoint_dir),
            device=device,
            z_range=z_range,
            local_regions=local_regions,
            filter_grasps=filter_grasps,
            skip_border_objects=skip_border_objects,
            segmap_id=segmap_id,
            max_segments=max_segments,
            forward_passes=forward_passes,
            first_thres=first_thres,
            second_thres=second_thres,
            num_samples=num_samples,
            max_farthest_points=max_farthest_points,
            seed=seed,
            service_timeout_s=service_timeout_s,
            mask_mode=mask_mode,
            depth_scale=depth_scale,
        )
        config.validate()
        return config

    def validate(self) -> None:
        if self.z_range[0] >= self.z_range[1]:
            raise ConfigError(f"z_range must be [min, max], got {self.z_range}")
        if self.segmap_id < 0:
            raise ConfigError(f"segmap_id must be >= 0, got {self.segmap_id}")
        if self.max_segments < 1:
            raise ConfigError(f"max_segments must be >= 1, got {self.max_segments}")
        if self.forward_passes < 1:
            raise ConfigError(f"forward_passes must be >= 1, got {self.forward_passes}")
        if self.num_samples < 1:
            raise ConfigError(f"num_samples must be >= 1, got {self.num_samples}")
        if self.max_farthest_points < 1:
            raise ConfigError(
                f"max_farthest_points must be >= 1, got {self.max_farthest_points}"
            )
        if not (0.0 <= self.first_thres <= 1.0):
            raise ConfigError(f"first_thres must be in [0, 1], got {self.first_thres}")
        if not (0.0 <= self.second_thres <= 1.0):
            raise ConfigError(
                f"second_thres must be in [0, 1], got {self.second_thres}"
            )
        if self.depth_scale <= 0.0:
            raise ConfigError(f"depth_scale must be positive, got {self.depth_scale}")
        if self.service_timeout_s <= 0.0:
            raise ConfigError(
                f"service_timeout_s must be positive, got {self.service_timeout_s}"
            )
        if self.device not in ("cuda", "cpu", "auto"):
            raise ConfigError(f"device must be cuda, cpu or auto, got {self.device!r}")
        if self.mask_mode not in ("labels", "binary"):
            raise ConfigError(
                f"mask_mode must be labels or binary, got {self.mask_mode!r}"
            )
        if not self.model_config_path.is_file():
            raise ConfigError(f"model_config not found: {self.model_config_path}")
        if not self.checkpoint_path.is_file():
            raise ConfigError(f"checkpoint not found: {self.checkpoint_path}")


def _resolve_model_config(value: str) -> Path:
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    config_dir = Path(
        os.environ.get("GRASPNET_CONFIG_DIR", Path.cwd() / "config")
    ).expanduser()
    return (config_dir / path).resolve()


def _resolve_checkpoint_dir(value: str) -> Path:
    override = os.environ.get("GRASPNET_CHECKPOINT_DIR")
    if override:
        return Path(override).expanduser().resolve()
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    root = Path(os.environ.get("CONTACT_GRASPNET_ROOT", Path.cwd())).expanduser()
    return (root / path).resolve()
