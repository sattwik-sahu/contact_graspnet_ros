"""Loading and validation of the Contact-GraspNet model configuration."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

import yaml

from .types import ModelDefaults

_REQUIRED_SECTIONS = ("DATA", "MODEL", "TEST")
_REQUIRED_DATA_KEYS = ("raw_num_points", "num_point", "gripper_width", "input_normals")
_REQUIRED_MODEL_KEYS = ("model", "bin_offsets")
_REQUIRED_TEST_KEYS = (
    "first_thres",
    "second_thres",
    "extra_opening",
    "max_farthest_points",
    "num_samples",
    "filter_thres",
)


class ModelConfigError(ValueError):
    """Raised when ``model.yaml`` is missing or malformed."""


def load_model_config(path: str | Path) -> dict[str, Any]:
    """Load the model YAML as a plain dictionary.

    The wrapper intentionally uses its own ``config/model.yaml`` instead of the
    checkpoint directory's copy so that all pipeline configuration lives in one
    place.  The file is a copy of the upstream PyTorch port configuration.
    """

    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise ModelConfigError(f"Model config not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle)

    if not isinstance(config, Mapping):
        raise ModelConfigError(f"Model config must be a YAML mapping: {config_path}")

    result = dict(config)
    _validate_config(result, config_path)
    return result


def _validate_config(config: Mapping[str, Any], path: Path) -> None:
    missing = [section for section in _REQUIRED_SECTIONS if section not in config]
    if missing:
        raise ModelConfigError(f"{path}: missing sections {missing}")

    data = config["DATA"]
    model = config["MODEL"]
    test = config["TEST"]
    if (
        not isinstance(data, Mapping)
        or not isinstance(model, Mapping)
        or not isinstance(test, Mapping)
    ):
        raise ModelConfigError(f"{path}: DATA, MODEL and TEST must be mappings")

    missing_data = [key for key in _REQUIRED_DATA_KEYS if key not in data]
    missing_model = [key for key in _REQUIRED_MODEL_KEYS if key not in model]
    missing_test = [key for key in _REQUIRED_TEST_KEYS if key not in test]
    if missing_data or missing_model or missing_test:
        raise ModelConfigError(
            f"{path}: missing keys "
            f"DATA={missing_data}, MODEL={missing_model}, TEST={missing_test}"
        )

    if bool(data["input_normals"]):
        raise ModelConfigError(
            f"{path}: input_normals=true is not supported by the depth-based ROS wrapper"
        )

    if int(data["raw_num_points"]) <= 0 or int(data["num_point"]) <= 0:
        raise ModelConfigError(f"{path}: raw_num_points and num_point must be positive")

    if float(data["gripper_width"]) <= 0.0:
        raise ModelConfigError(f"{path}: gripper_width must be positive")


def resolve_model_defaults(config: Mapping[str, Any]) -> ModelDefaults:
    """Extract the values the engine needs for inference and selection."""

    data = config["DATA"]
    test = config["TEST"]
    return ModelDefaults(
        raw_num_points=int(data["raw_num_points"]),
        num_point=int(data["num_point"]),
        gripper_width=float(data["gripper_width"]),
        extra_opening=float(test["extra_opening"]),
        first_thres=float(test["first_thres"]),
        second_thres=float(test["second_thres"]),
        filter_thres=float(test["filter_thres"]),
        max_farthest_points=int(test["max_farthest_points"]),
        num_samples=int(test["num_samples"]),
        use_farthest_point=bool(data.get("use_farthest_point", False)),
        center_to_tip=float(test.get("center_to_tip", 0.0)),
        input_normals=bool(data["input_normals"]),
    )


def apply_runtime_overrides(
    config: dict[str, Any],
    *,
    first_thres: float | None = None,
    second_thres: float | None = None,
    max_farthest_points: int | None = None,
    num_samples: int | None = None,
) -> dict[str, Any]:
    """Return a copy of the config with runtime selection knobs overridden.

    Architecture values are never overridden here.  Only the fields exposed by
    the ROS interface may change at runtime.
    """

    result = dict(config)
    test = dict(result.get("TEST", {}))
    if first_thres is not None:
        test["first_thres"] = float(first_thres)
    if second_thres is not None:
        test["second_thres"] = float(second_thres)
    if max_farthest_points is not None:
        test["max_farthest_points"] = int(max_farthest_points)
    if num_samples is not None:
        test["num_samples"] = int(num_samples)
    result["TEST"] = test
    return result
