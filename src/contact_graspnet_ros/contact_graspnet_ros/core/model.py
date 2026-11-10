"""Torch model construction, checkpoint loading and seed handling.

This module deliberately does **not** import the upstream ``GraspEstimator``.
That class contains dead TensorFlow methods and leaves the model in training
mode.  We use only the upstream ``ContactGraspnet`` module class and own the
inference orchestration in :mod:`contact_graspnet_ros.core.engine`.
"""

from __future__ import annotations

import os
import random
import sys
import warnings
from functools import lru_cache
from pathlib import Path
from typing import Any

import numpy as np

from .config import ModelConfigError

_TORCH_IMPORT_ERROR: Exception | None = None
_TORCH = None


def _require_torch():
    """Import Torch lazily so geometry tests do not require it."""

    global _TORCH, _TORCH_IMPORT_ERROR
    if _TORCH is not None:
        return _TORCH
    try:
        import torch
    except Exception as exc:  # pragma: no cover - depends on environment
        _TORCH_IMPORT_ERROR = exc
        raise RuntimeError(
            "PyTorch is required for the Contact-GraspNet backend. "
            "Install it with Pixi (see pixi.toml) or pip."
        ) from exc
    _TORCH = torch
    return torch


def _submodule_root() -> Path:
    """Locate the read-only ``contact_graspnet_pytorch`` submodule."""

    env_root = os.environ.get("CONTACT_GRASPNET_ROOT")
    candidates: list[Path] = []
    if env_root:
        candidates.append(Path(env_root) / "third_party" / "contact_graspnet_pytorch")
    candidates.extend(
        parent / "third_party" / "contact_graspnet_pytorch"
        for parent in Path(__file__).resolve().parents
    )

    for candidate in candidates:
        if (candidate / "contact_graspnet_pytorch" / "contact_graspnet.py").is_file():
            return candidate

    raise ModelConfigError(
        "Could not locate third_party/contact_graspnet_pytorch. "
        "Set CONTACT_GRASPNET_ROOT to the repository root, or run from the "
        "Pixi/Docker environment where it is set."
    )


def _ensure_submodule_on_path() -> Path:
    root = _submodule_root()
    for path in (root.parent, root, root / "Pointnet_Pointnet2_pytorch"):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)
    return root


@lru_cache(maxsize=1)
def _contact_graspnet_class():
    """Import the upstream model class and return a patched subclass.

    The upstream ``build_6d_grasp`` calls ``torch.cross`` without a dimension,
    which is deprecated and may become an error.  The subclass override keeps
    the submodule untouched while using an explicit dimension.
    """

    torch = _require_torch()
    _ensure_submodule_on_path()
    try:
        from contact_graspnet_pytorch.contact_graspnet import ContactGraspnet
    except Exception as exc:  # pragma: no cover - depends on submodule
        raise RuntimeError(
            "Failed to import contact_graspnet_pytorch.contact_graspnet. "
            "Check that the submodule is initialized and that its Python "
            "dependencies (torch, numpy, scipy, trimesh, opencv, pillow) are installed."
        ) from exc

    class PatchedContactGraspnet(ContactGraspnet):  # type: ignore[misc, valid-type]
        """Upstream model with a dimension-safe 6-DoF grasp builder."""

        def build_6d_grasp(
            self,
            approach_dirs,
            base_dirs,
            contact_pts,
            thickness,
            use_torch: bool = False,
            gripper_depth: float = 0.1034,
        ):
            if not use_torch:
                raise NotImplementedError(
                    "PatchedContactGraspnet expects Torch tensors"
                )

            if thickness.ndim == 2:
                thickness = thickness.unsqueeze(2)
            if thickness.ndim == 4:
                thickness = thickness.squeeze(-1)
            if thickness.ndim != 3:
                raise ValueError(
                    f"thickness must be (B,N,1), got {tuple(thickness.shape)}"
                )

            grasp_rotation = torch.stack(
                [
                    base_dirs,
                    torch.cross(approach_dirs, base_dirs, dim=2),
                    approach_dirs,
                ],
                dim=3,
            )
            grasp_translation = (
                contact_pts
                + (thickness / 2.0) * base_dirs
                - float(gripper_depth) * approach_dirs
            ).unsqueeze(3)
            ones = torch.ones(
                (contact_pts.shape[0], contact_pts.shape[1], 1, 1),
                dtype=contact_pts.dtype,
                device=contact_pts.device,
            )
            zeros = torch.zeros(
                (contact_pts.shape[0], contact_pts.shape[1], 1, 3),
                dtype=contact_pts.dtype,
                device=contact_pts.device,
            )
            homogeneous = torch.cat([zeros, ones], dim=3)
            return torch.cat(
                [torch.cat([grasp_rotation, grasp_translation], dim=3), homogeneous],
                dim=2,
            )

    return PatchedContactGraspnet


def resolve_device(device: str | None) -> Any:
    """Resolve ``cuda``, ``cpu`` or ``auto`` to a ``torch.device``."""

    torch = _require_torch()
    requested = (device or "auto").strip().lower()
    if requested == "auto":
        return torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    if requested == "cuda":
        if not torch.cuda.is_available():
            raise RuntimeError("device='cuda' was requested but CUDA is not available")
        return torch.device("cuda:0")
    if requested == "cpu":
        return torch.device("cpu")
    raise ValueError(f"Unsupported device {device!r}; expected 'cuda', 'cpu' or 'auto'")


def build_model(config: dict[str, Any], device: Any):
    """Construct the patched Contact-GraspNet model and move it to ``device``."""

    model_class = _contact_graspnet_class()
    model = model_class(config, device)
    model.to(device)
    return model


def load_checkpoint(model, checkpoint_path: str | Path, device: Any) -> None:
    """Load a PyTorch state dict and switch the model to inference mode."""

    torch = _require_torch()
    path = Path(checkpoint_path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"Contact-GraspNet checkpoint not found: {path}")

    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            checkpoint = torch.load(path, map_location=device, weights_only=True)
    except TypeError:
        # Older Torch versions do not support weights_only.
        checkpoint = torch.load(path, map_location=device)
    except Exception:  # noqa: BLE001 - fallback for legacy checkpoints
        # The bundled checkpoint contains NumPy metadata (epoch/loss scalars)
        # that newer Torch versions reject under weights_only=True.  The file
        # is a trusted local artifact, so retry with the legacy loader.
        warnings.warn(
            f"Checkpoint {path} contains NumPy metadata; loading with the legacy "
            "pickle loader (trusted local checkpoint).",
            RuntimeWarning,
            stacklevel=2,
        )
        checkpoint = torch.load(path, map_location=device, weights_only=False)

    if isinstance(checkpoint, dict):
        if "model" in checkpoint:
            state_dict = checkpoint["model"]
        elif "state_dict" in checkpoint:
            state_dict = checkpoint["state_dict"]
        else:
            state_dict = checkpoint
    else:
        state_dict = checkpoint

    if hasattr(state_dict, "state_dict"):
        state_dict = state_dict.state_dict()

    if not isinstance(state_dict, dict):
        raise TypeError(f"Checkpoint {path} does not contain a state dict")

    if any(str(key).startswith("module.") for key in state_dict):
        state_dict = {
            str(key).removeprefix("module."): value for key, value in state_dict.items()
        }

    missing, unexpected = model.load_state_dict(state_dict, strict=False)
    if missing or unexpected:
        raise RuntimeError(
            f"Checkpoint {path} is incompatible with the model. "
            f"Missing keys: {list(missing)[:5]} ({len(missing)} total); "
            f"unexpected keys: {list(unexpected)[:5]} ({len(unexpected)} total)."
        )

    model.eval()
    for parameter in model.parameters():
        parameter.requires_grad_(False)


def set_seed(seed: int) -> None:
    """Seed Python, NumPy, Torch and CUDA where available."""

    seed = int(seed)
    random.seed(seed)
    np.random.seed(seed)
    torch = _require_torch()
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)
