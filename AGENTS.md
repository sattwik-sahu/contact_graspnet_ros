# AGENTS.md — repository context for coding agents

This file is the entry point for AI coding agents working in this repository.
Read it before changing anything. A human-readable version is published at
[docs/agent-guide.md](docs/agent-guide.md).

## 1. What this repository is

A **service-only ROS 2 wrapper** around the PyTorch port of
[Contact-GraspNet](https://github.com/elchun/contact_graspnet_pytorch).

```text
depth image + camera intrinsics + optional segmentation mask
        -> Contact-GraspNetEngine.infer(...)
        -> GraspProposalArray (6-DoF poses, scores, widths, contacts)
```

It is a **perception** component. It does not plan, execute, or simulate robot
motion. MoveIt (or any other planner) lives in a downstream package.

- ROS 2 Jazzy is first-class; Humble is a secondary target.
- Python 3.12, PyTorch, NumPy 2.x, Pixi-managed environments.
- The model weights are not in Git; they are mounted or downloaded into the
  vendored port's checkpoint directory.

## 2. Non-negotiable rules

1. **Keep `core/` ROS-free.** No `rclpy`, `cv_bridge`, `sensor_msgs`, or
   `geometry_msgs` imports under
   `src/contact_graspnet_ros/contact_graspnet_ros/core/`. It is intentionally
   testable without a sourced ROS environment.
2. **Treat `third_party/contact_graspnet_pytorch/` as read-only.** It is a Git
   submodule. Do not edit files in it and do not commit changes inside it.
   Fixes and orchestration belong in `src/contact_graspnet_ros/`.
3. **Do not reimplement inference in the ROS layer.** Load the model once with
   `ContactGraspNetEngine`, call `infer(...)` / `infer_pointclouds(...)`, and
   convert the returned dataclasses to ROS messages.
4. **Configuration lives in `config/` and ROS parameters.** Never hard-code
   thresholds, paths, device, frame names or tool offsets in Python.
5. **Preserve the output-frame contract.** With defaults, poses are
   gripper-base poses in the camera optical frame. `target_frame` selects a TF
   frame; `tcp_offset` / `tcp_rpy` map the network gripper frame to the
   downstream tool/EE frame. Transform the contact point with the same TF as
   the pose, but never with the tool offset.
6. **No MoveIt dependency here.** Expose a clean `PoseStamped`-ready pose;
   downstream chooses the planning frame and tip link.
7. **Update docs with behaviour changes.** Adding a parameter, interface field,
   or output semantic means updating `docs/configuration.md`,
   `docs/ros-interfaces.md`, and/or `docs/grasp-frames.md`.

## 3. Repository map

```text
AGENTS.md                         this file (agent entry point)
zensical.toml                     documentation site config (Zensical)
docs-requirements.txt             docs/banner build deps (zensical, matplotlib)
docs/                             documentation source + agent-guide.md,
                                  example-output.md and assets/banner.png
config/
  graspnet.yaml                   ROS node parameters (runtime)
  model.yaml                      model architecture + defaults (authoritative)
src/contact_graspnet_msgs/
  srv/GenerateGrasps.srv          request/response contract
  msg/GraspProposal.msg           one grasp
  msg/GraspProposalArray.msg      response payload + frame header
src/contact_graspnet_ros/
  contact_graspnet_ros/
    node.py                       rclpy service node, TF, message conversion
    config.py                     ROS parameter parsing/validation
    conversions.py                ROS <-> NumPy/Typed conversions
    core/
      engine.py                   ROS-free ContactGraspNetEngine
      types.py                    CameraIntrinsics, InferenceOptions,
                                  GraspProposal, GraspResult, ModelDefaults
      geometry.py                 point clouds, cropping, grasp selection
      model.py                    model build, checkpoint load, device/seed
      config.py                   model.yaml loading and defaults
      transforms.py               pure-NumPy rigid transforms
    test/                         pytest suite
  launch/                         launch files
scripts/
  run_inference.py                ROS-free engine smoke script
  call_grasp_service.py           ROS service client example
  demo_from_mesh.py               download a mesh, sample points, run inference
  render_banner.py                render 3-D grasp plot + README/docs banner
  activate.sh                     Pixi activation hook (env vars + install setup)
docker/                           Dockerfile, entrypoint, include-able compose file
docker-compose.yaml               root compose project (+ Zenoh router)
.github/workflows/                docs (Zensical) and Docker build CI
third_party/contact_graspnet_pytorch/   vendored, read-only upstream port
```

## 4. Data flow and key APIs

### ROS path

1. `node.py::_handle_generate_grasps` calls `image_to_depth`,
   `camera_info_to_intrinsics`, and `image_to_mask` from `conversions.py`.
2. `_request_options` builds `InferenceOptions` from request fields + node
   parameters.
3. `ContactGraspNetEngine.infer(depth, intrinsics, mask, options)` returns a
   `GraspResult`.
4. `_resolve_output_frame` looks up `T_target_camera` with `tf2_ros.Buffer`
   when `target_frame` differs from the camera frame.
5. `_to_message` applies
   `T_output_tool = T_target_camera @ proposal.pose @ T_grasp_tool`, transforms
   the contact point with `T_target_camera`, and fills `GraspProposalArray`.

### Core API

```python
engine = ContactGraspNetEngine(
    model_config_path="config/model.yaml",
    checkpoint_path="third_party/.../checkpoints/model.pt",
    device="cuda",
    seed=0,
)
result = engine.infer(depth, intrinsics, mask, InferenceOptions())
# result.proposals[i].pose        (4, 4) float32
# result.proposals[i].score       float
# result.proposals[i].width       metres
# result.proposals[i].contact_point  (3,) float32
# result.proposals[i].segment_id   int
```

`result.proposals` is sorted by descending score and may be empty (with
warnings) rather than raising.

The ROS-free alternative is
`engine.infer_pointclouds(points_nx3, segment_nx3_or_none, options)`.

### Pose semantics

- `+X` = finger-closing axis, `+Z` = approach axis, `+Y` = `Z x X`.
- Translation = `contact + width/2 * X - 0.1034 * Z`.
- `width` is metres, quantized into 10 bins over `[0, 0.08]`, plus a 5 mm
  margin, clipped to 0.08 m.
- `score` is a ranking confidence, not a calibrated success probability.

## 5. Common changes and where they belong

| Task | Where |
|---|---|
| Add/rename a service field | `src/contact_graspnet_msgs/srv/GenerateGrasps.srv`, `node.py`, `docs/ros-interfaces.md`, client script, rebuild the msgs package |
| Add a node parameter | `config.py` (declare + validate), `config/graspnet.yaml`, `docs/configuration.md`, possibly launch args |
| Add a runtime inference option | `core/types.py::InferenceOptions`, pass through `node.py::_request_options`, document |
| Change message semantics/frame | `.msg` comments, `node.py::_to_message`, `docs/inference-contract.md`, `docs/grasp-frames.md` |
| Add pure math/geometry helper | `core/geometry.py` or `core/transforms.py` + a unit test in `src/contact_graspnet_ros/test/` |
| Add a ROS conversion helper | `conversions.py` |
| Add a docs page | `docs/`, add it to `nav` in `zensical.toml`, link it from `docs/index.md` and README where useful |
| Change Docker/network setup | `docker/compose.contact_graspnet.yaml`, root `docker-compose.yaml`, `docs/docker.md`, `docs/deployment.md` |

After changing a `.srv` or `.msg`, **rebuild** so generated interfaces exist:

```bash
pixi run -e jazzy build
```

## 6. Commands

```bash
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy lint          # ruff
pixi run -e jazzy typecheck     # mypy on core/
pixi run -e jazzy launch
pixi run -e jazzy zenohd        # terminal 1 (Jazzy/Zenoh)
```

ROS-free inference check:

```bash
pixi run -e jazzy python scripts/run_inference.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --device cuda --segmap-id 1
```

Service client example:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1 --target-frame base_link
```

Docs:

```bash
pip install -r docs-requirements.txt
zensical serve
zensical build --clean --strict
```

Docker:

```bash
docker compose build
docker compose up
docker compose config
```

### Environment notes

- Pixi env: `.pixi/envs/jazzy/` (Python 3.12). Use its `bin/python` when
  running ad-hoc checks.
- `scripts/activate.sh` sets `CONTACT_GRASPNET_ROOT`, `GRASPNET_CONFIG_DIR`,
  and `PYTHONPATH` for the vendored port.
- Overrides: `GRASPNET_CONFIG_DIR`, `GRASPNET_CHECKPOINT_DIR`,
  `CONTACT_GRASPNET_ROOT`.
- Checkpoint:
  `third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt`.
- `torch.load` compatibility, `np.in1d` -> `np.isin`, and avoided
  `pyrender`/`open3d` imports already live in `core/`; do not reintroduce the
  upstream file-driven path.

### Test runner caveat

`pixi run -e jazzy test` currently collects 0 tests because of a pre-existing
interaction between pytest 9 and the installed `ament_*` plugins (an untouched
test file behaves the same). To actually run the suite:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .pixi/envs/jazzy/bin/python -m pytest -q src/contact_graspnet_ros/test
```

Prefer adding small, ROS-free unit tests under `src/contact_graspnet_ros/test/`.

## 7. Style

- Type-annotate public APIs; keep functions small and testable.
- Use `ruff` conventions; run `mypy` on `core/`.
- Use frozen dataclasses for value objects.
- Docstrings for public functions/classes.
- Docs use relative Markdown links (e.g. `[Configuration](configuration.md)`).
- Conventional commit subjects: `feat:`, `fix:`, `docs:`, `refactor:`.

## 8. Gotchas

- Depth must be metres. `32FC1` is metres; `16UC1`/`mono16` is scaled by
  `depth_scale` (default 0.001).
- Masks must match depth size; `0` = background. `local_regions` and
  `filter_grasps` require a mask.
- `segmap_id=0` processes all positive labels (up to `max_segments`); a
  positive id processes only that label.
- `z_range` is camera-z distance, not world height; `min < max`.
- TF lookup uses the depth stamp; a zero stamp means "latest". A dedicated TF
  listener node/thread avoids blocking the service callback.
- `TransformListener` is created with `node=None, spin_thread=True` on purpose;
  do not attach it to the main executor or callbacks can be double-spun.
- Requests are serialized by a lock; a concurrent call returns
  `success=false` with a `busy` message after `service_timeout_s`.
- Engine/model is not thread-safe; keep the lock.
- Compose sets both `network_mode: host` and `ipc: host` on every service; keep
  them for ROS discovery and shared memory.
- README, docs and `AGENTS.md` should describe the same behaviour; do not let
  them drift.
