from pathlib import Path

import pytest
from contact_graspnet_ros.core.config import (
    ModelConfigError,
    load_model_config,
    resolve_model_defaults,
)

REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_CONFIG = REPO_ROOT / "config" / "model.yaml"


def test_load_model_config_and_defaults() -> None:
    config = load_model_config(MODEL_CONFIG)
    defaults = resolve_model_defaults(config)

    assert defaults.raw_num_points == 20000
    assert defaults.num_point == 2048
    assert defaults.first_thres == pytest.approx(0.15)
    assert defaults.second_thres == pytest.approx(0.15)
    assert defaults.input_normals is False


def test_missing_config_raises() -> None:
    with pytest.raises(ModelConfigError):
        load_model_config("/definitely/not/a/config.yaml")
