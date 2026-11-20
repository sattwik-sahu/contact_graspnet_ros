# Agent Guide

This page tells AI coding agents (and humans onboarding them) how to work in
this repository. The canonical, machine-readable entry point is
[`AGENTS.md`](https://github.com/sattwik-sahu/contact_graspnet_ros/blob/main/AGENTS.md)
at the repository root; this page is the published, human-friendly version.

## What this project is

A **service-only ROS 2 wrapper** for the PyTorch port of Contact-GraspNet:

```text
depth + camera intrinsics + optional segmentation mask
        -> ContactGraspNetEngine.infer(...)
        -> GraspProposalArray (poses, scores, widths, contacts)
```

The repository stops at perception. Motion planning (MoveIt or otherwise)
belongs in a downstream package. See [Architecture](architecture.md) and
[Inference Contract](inference-contract.md).

## Rules agents must follow

1. Keep `core/` **ROS-free** — no `rclpy`, `cv_bridge` or ROS message imports.
2. Treat `third_party/contact_graspnet_pytorch/` as a **read-only submodule**.
3. Do not reimplement inference in the ROS layer; use `ContactGraspNetEngine`.
4. Keep configuration in `config/` and ROS parameters, not hard-coded values.
5. Preserve the frame contract: default output is the gripper-base pose in the
   camera optical frame; `target_frame` selects a TF frame and `tcp_offset` /
   `tcp_rpy` map to the tool frame. See [Grasp Frames](grasp-frames.md).
6. Do not add a MoveIt dependency here.
7. Update the docs when behaviour, interfaces or defaults change.

## Where changes belong

| Change | Files |
|---|---|
| Service field | `src/contact_graspnet_msgs/srv/GenerateGrasps.srv`, `node.py`, client script, [ROS Interfaces](ros-interfaces.md) |
| ROS parameter | `config.py`, `config/graspnet.yaml`, [Configuration](configuration.md) |
| Inference option | `core/types.py` (`InferenceOptions`), `node.py`, docs |
| Pure math helper | `core/transforms.py` or `core/geometry.py` + a unit test |
| ROS conversion | `conversions.py` |
| New documentation page | `docs/`, `nav` in `zensical.toml`, [index](index.md) |
| Docker / Compose | `docker/`, `docker-compose.yaml`, [Docker guide](docker.md) |

Remember to run `pixi run -e jazzy build` after editing an interface file so
generated message code is up to date.

## Commands

```bash
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy lint
pixi run -e jazzy typecheck
pixi run -e jazzy launch
```

ROS-free smoke test:

```bash
pixi run -e jazzy python scripts/run_inference.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --device cuda --segmap-id 1
```

Service client:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1 --target-frame base_link
```

Documentation:

```bash
pip install -r docs-requirements.txt
zensical serve
zensical build --clean --strict
```

### Test-runner caveat

`pixi run -e jazzy test` currently collects 0 tests due to a pre-existing
pytest 9 / `ament_*` plugin interaction. Run the suite with:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .pixi/envs/jazzy/bin/python -m pytest -q src/contact_graspnet_ros/test
```

## Style and review checklist

- Type annotations on public APIs; frozen dataclasses for value objects.
- `ruff` and `mypy` (on `core/`) must pass.
- Small, testable functions; public helpers get docstrings.
- Docs use relative Markdown links.
- Conventional commit subjects (`feat:`, `fix:`, `docs:`).
- No changes inside `third_party/`.
- New parameters documented in [Configuration](configuration.md); new
  interfaces in [ROS Interfaces](ros-interfaces.md).

## Further reading

- [Getting Started](getting-started.md)
- [Architecture](architecture.md)
- [Inference Contract](inference-contract.md)
- [ROS Interfaces](ros-interfaces.md)
- [Grasp Frames and Downstream Planning](grasp-frames.md)
- [Docker and Docker Compose](docker.md)
- [Deployment](deployment.md)
- [Contributing](contributing.md)
