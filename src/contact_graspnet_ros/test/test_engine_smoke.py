from pathlib import Path

import pytest

torch = pytest.importorskip("torch")

from contact_graspnet_ros.core.config import load_model_config
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.model import build_model

REPO_ROOT = Path(__file__).resolve().parents[3]
MODEL_CONFIG = REPO_ROOT / "config" / "model.yaml"
CHECKPOINT = (
    REPO_ROOT
    / "third_party"
    / "contact_graspnet_pytorch"
    / "checkpoints"
    / "contact_graspnet"
    / "checkpoints"
    / "model.pt"
)


@pytest.mark.skipif(not CHECKPOINT.is_file(), reason="checkpoint not available")
def test_engine_constructs_and_loads_checkpoint() -> None:
    engine = ContactGraspNetEngine(
        model_config_path=MODEL_CONFIG,
        checkpoint_path=CHECKPOINT,
        device="cpu",
        seed=0,
    )
    try:
        assert engine.defaults.raw_num_points == 20000
        assert engine.device_name == "cpu"
    finally:
        engine.close()


def test_patched_model_forward_smoke() -> None:
    config = load_model_config(MODEL_CONFIG)
    # The upstream PointNet2 query_ball_point implementation requires the
    # number of input points at each layer to be at least nsample.  Keep the
    # reduced test config above those minima.
    config["DATA"]["raw_num_points"] = 512
    config["DATA"]["num_point"] = 256
    config["DATA"]["ndataset_points"] = 512
    config["MODEL"]["pointnet_sa_modules_msg"][0]["npoint"] = 256
    config["MODEL"]["pointnet_sa_modules_msg"][1]["npoint"] = 128
    config["MODEL"]["pointnet_sa_modules_msg"][2]["npoint"] = 8

    device = torch.device("cpu")
    model = build_model(config, device)
    model.eval()
    with torch.inference_mode():
        prediction = model(torch.randn(1, 512, 3))

    assert prediction["pred_grasps_cam"].shape == (1, 256, 4, 4)
    assert prediction["pred_scores"].shape == (1, 256, 1)
    assert prediction["pred_points"].shape == (1, 256, 3)
    assert prediction["offset_pred"].shape[0] == 1
