---
layout: page
title: Design and PRD
description: Product requirements and design record for the wrapper.
permalink: /design/
---

# Design and PRD

> This is the historical design record. For task-oriented documentation, start with [Getting Started]({{ '/getting-started/' | relative_url }}).
>
> **Delivered scope:** the implemented wrapper is service-only. Topic subscriptions, output streams, RViz markers, TF output transforms and diagnostics were removed by decision; the sections below describe the earlier design and remain for context.

**Status:** Implemented for Jazzy; Humble and CI image build pending verification  
**Revision:** 2 — Python-only; supersedes the C++/pybind11 architecture  
**Branch:** `dev/feature/ros2-jazzy`  
**Note:** This PRD is kept in the repository as the design record. The current
implementation follows it; remaining work is tracked in the milestones.

---

## 1. Executive summary

Build a modular, Dockerized ROS 2 wrapper around the vendored Contact-GraspNet PyTorch port at `third_party/contact_graspnet_pytorch`.

The wrapper is a **Python `rclpy` package** plus a small message package:

1. `contact_graspnet_msgs` — ROS 2 message/service definitions (`ament_cmake`).
2. `contact_graspnet_ros` — Python ROS 2 node (`ament_python`) containing:
   - a typed, ROS-free `core/` engine (PyTorch inference, geometry, config),
   - the `rclpy` node, conversions, TF2, QoS, markers and diagnostics.

The submodule is **read-only**. We use only its `ContactGraspnet` model class and its `model.pt` checkpoint. All orchestration, preprocessing, postprocessing, typing and bug fixes live in our package.

Jazzy is first-class: Python 3.12, PyTorch 2.x, CUDA 12.x, Zenoh. Humble is a secondary build target. Pixi and Docker are first-class.

C++/pybind11 is explicitly dropped for now. The engine boundary remains narrow so a future LibTorch/TensorRT backend could be added without changing the ROS interface.

---

## 2. Current state

- Branch: `dev/feature/ros2-jazzy`.
- Submodule: `third_party/contact_graspnet_pytorch` pinned at `2d71da4e50a04aa353352d1cae99f20f7022145b`.
- Submodule contents include:
  - `contact_graspnet_pytorch/` (model + training code)
  - `Pointnet_Pointnet2_pytorch/` (pure-PyTorch PointNet2, no CUDA extension)
  - `checkpoints/contact_graspnet/config.yaml`
  - `checkpoints/contact_graspnet/checkpoints/model.pt` (26.6 MB)
- Existing empty placeholders:
  - `pixi.toml`
  - `README.md`
  - `docker/Dockerfile`
  - `docker/compose.contact_graspnet.yaml`
  - `docker-compose.yaml`
  - `src/`
  - `config/`
  - `docs/`
- `.gitmodules` points at `https://github.com/elchun/contact_graspnet_pytorch.git`.

The submodule's `contact_graspnet_env.yml` is obsolete (Python 3.9, Torch 2.0.1, CUDA 11.7). It is not used. Pixi owns the environment.

---

## 3. Goals and non-goals

### 3.1 Goals

1. Provide a ROS 2 Python node that accepts depth + camera intrinsics + optional segmentation mask and publishes typed grasp proposals.
2. Keep the ML engine modular, typed, ROS-free and independently unit-testable.
3. Never modify the submodule; accommodate its bugs externally.
4. Support both topic mode and service mode.
5. Provide a single configuration folder for ROS, model and deployment settings.
6. Support Jazzy first-class and Humble secondarily.
7. Use Pixi for reproducible host development and Docker for deployment.
8. Provide an include-able Compose service and a standalone root Compose file.
9. Use Zenoh as the default RMW for Jazzy, while keeping the code RMW-agnostic.
10. Make the ROS interface stable enough to integrate into other pipelines without touching the engine.

### 3.2 Non-goals

- Do not retrain or fine-tune the model.
- Do not modify the submodule or convert the official NVIDIA TensorFlow checkpoint.
- Do not implement a C++/pybind11/LibTorch node in this revision.
- Do not implement MoveIt-specific grasp messages in the core package.
- Do not implement segmentation (UOIS/FastSAM/etc.); segmentation is an input.
- Do not implement a full HTTP server in v1. An optional HTTP bridge may reuse `core/engine.py` later.
- Do not support Windows/macOS.
- Do not guarantee Humble runtime parity in v1; it is a build/secondary target.

---

## 4. Architecture

### 4.1 Component diagram

```text
┌──────────────────────── contact_graspnet_ros (Python, rclpy) ────────────────────────┐
│                                                                                     │
│  depth ─┐                                                                           │
│  info  ─┼─ message_filters ApproximateTime ─► node callback ─► engine.infer()        │
│  mask  ─┘                                       (bounded by lock)                    │
│                                                                       │             │
│                                                                       ▼             │
│                                                        core/ (ROS-free, typed)      │
│                                                        ├─ config.py  (model.yaml)   │
│                                                        ├─ geometry.py (depth2pc...) │
│                                                        ├─ model.py    (ContactGraspnet)│
│                                                        ├─ types.py    (dataclasses) │
│                                                        └─ engine.py   (load + infer)│
│                                                                       │             │
│  GraspProposalArray ◄─────────────────────────────────────────────────┘             │
│  PointCloud2 / MarkerArray / DiagnosticArray                                        │
│  GenerateGrasps / GenerateGraspsFromPointClouds services                            │
└─────────────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
                     third_party/contact_graspnet_pytorch (read-only)
                     ├─ contact_graspnet_pytorch/contact_graspnet.py
                     ├─ Pointnet_Pointnet2_pytorch/
                     └─ checkpoints/contact_graspnet/checkpoints/model.pt
```

### 4.2 ROS-free core boundary

`contact_graspnet_ros/core/` must not import `rclpy` or any ROS message package. This gives:

- fast pytest unit tests without sourcing ROS,
- reuse from an HTTP bridge or CLI later,
- a clean seam for a future LibTorch/ONNX backend.

The node imports the engine:

```python
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.types import CameraIntrinsics, InferenceOptions
```

`contact_graspnet_ros/__init__.py` must not import `rclpy`; only `node.py` and the console entry point do.

### 4.3 Submodule policy

The submodule is treated as a read-only model dependency.

We use:

- `contact_graspnet_pytorch.contact_graspnet.ContactGraspnet` (model class only).
- `checkpoints/contact_graspnet/checkpoints/model.pt` (weights).
- `checkpoints/contact_graspnet/config.yaml` as the source for `config/model.yaml`.

We do **not** use:

- `contact_graspnet_pytorch.contact_grasp_estimator.GraspEstimator` (dead TF methods, missing `model.eval()`).
- `contact_graspnet_pytorch.checkpoints.CheckpointIO` (wrong error type, no `map_location`).
- `contact_graspnet_pytorch.data` (imports pyrender).
- `contact_graspnet_pytorch.inference` (imports Open3D visualisation).

External fixes implemented in our core:

| Issue in submodule | External fix |
|---|---|
| Model left in training mode | `model.eval()` after construction |
| No `torch.inference_mode()` | Wrap forward pass |
| `torch.cross` without `dim` | Subclass override with `dim=2` |
| `not np.any(selection_idcs)` bug | `len(selection_idcs) == 0` |
| One-element outputs squeezed | Normalize with `reshape(-1, ...)` |
| `-1` key retained when filtering | Normalize/ignore in engine |
| `torch.load` without `map_location` | Load explicitly in our engine |
| `CheckpointIO` raises `FileExistsError` | Raise `FileNotFoundError` with a clear path |
| `data.py` imports pyrender | Copy pure geometry helpers into `geometry.py` |
| Training-only modules imported | Do not import them |

The copied geometry helpers are documented in `third_party/NOTICE` and remain under the upstream NVIDIA license.

### 4.4 Data flow

```text
depth (H,W f32, m) + P/K + optional mask
        │
        ▼
depth_to_point_cloud()
        │
        ├─ z_range crop
        ├─ per-label segment extraction
        └─ optional 3D local-region crop around target
        │
        ▼
regularize to raw_num_points (20k)
        │
        ▼
ContactGraspnet.forward() under torch.inference_mode()
        │  pred_grasps_cam (B,N,4,4)
        │  pred_scores    (B,N,1)
        │  pred_points    (B,N,3)
        │  offset_pred    (B,N,1)
        ▼
select_grasps() + filter_grasps()
        │
        ▼
typed GraspResult:
  poses (N,4,4), scores (N,), widths (N,), contacts (N,3), segment_ids (N,)
        │
        ▼
rclpy conversions:
  GraspProposalArray + PointCloud2 + MarkerArray
```

### 4.5 Frames and pose convention

- Input depth and point clouds are expected in the **camera optical frame** (`x` right, `y` down, `z` forward), which matches REP 103 optical frames and the model's OpenCV convention.
- Output poses are in the input frame by default.
- If `output_frame` is non-empty, the node transforms poses and contact points with TF2 at the sensor timestamp. If TF is unavailable, the frame is dropped and an error is reported; do not silently publish a wrong frame.
- The 4×4 pose is the **gripper base frame**, not the contact point or TCP:
  - columns: `[closing direction, cross, approach direction]`
  - `+Z` = approach
  - `+X` = finger-closing direction
  - translation = `contact + width/2 * base_dir - 0.1034 * approach_dir`
- `width` is the commanded finger opening in metres, clamped to `[0, 0.08]`.
- `score` is contact confidence in `[0, 1]`, not calibrated grasp success.

---

## 5. Packages and repository layout

```text
contact_graspnet_ros/
├── pixi.toml
├── pixi.lock
├── README.md
├── .dockerignore
├── docker/
│   ├── Dockerfile
│   ├── entrypoint.sh
│   └── compose.contact_graspnet.yaml
├── docker-compose.yaml                  # includes docker/compose.contact_graspnet.yaml
├── config/                              # single source of truth
│   ├── graspnet.yaml                    # ROS parameters
│   ├── model.yaml                       # model config (copy of submodule config)
│   ├── rviz/graspnet.rviz
│   └── zenoh/                           # optional router/session config
├── docs/
│   ├── architecture.md
│   ├── inference_contract.md
│   ├── ros_interfaces.md
│   └── deployment.md
├── src/
│   ├── contact_graspnet_msgs/           # ament_cmake
│   │   ├── msg/GraspProposal.msg
│   │   ├── msg/GraspProposalArray.msg
│   │   ├── srv/GenerateGrasps.srv
│   │   ├── srv/GenerateGraspsFromPointClouds.srv
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── contact_graspnet_ros/            # ament_python
│       ├── contact_graspnet_ros/
│       │   ├── __init__.py
│       │   ├── core/                    # ROS-free, typed
│       │   │   ├── __init__.py
│       │   │   ├── types.py
│       │   │   ├── config.py
│       │   │   ├── geometry.py
│       │   │   ├── model.py
│       │   │   └── engine.py
│       │   ├── node.py                  # rclpy node
│       │   ├── conversions.py           # ROS <-> numpy/poses
│       │   ├── markers.py               # MarkerArray
│       │   └── config.py                # ROS param parsing/validation
│       ├── launch/
│       │   ├── graspnet.launch.py
│       │   └── graspnet_zenoh.launch.py
│       ├── test/
│       │   ├── test_geometry.py
│       │   ├── test_config.py
│       │   ├── test_engine_smoke.py
│       │   └── test_ros_integration.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── third_party/
│   └── contact_graspnet_pytorch/        # git submodule, read-only
└── build/ install/ log/ .pixi/          # gitignored
```

### 5.1 Package responsibilities

| Package | Build type | Responsibility |
|---|---|---|
| `contact_graspnet_msgs` | `ament_cmake` | Messages and services |
| `contact_graspnet_ros` | `ament_python` | ROS node + ROS-free typed PyTorch engine |

The message package is separate because ROS 2 IDL generation needs `ament_cmake`. The wrapper itself is a single Python package.

---

## 6. Functional requirements

### FR-1 — Build and packages

- `colcon build` from the repo root builds both packages on Jazzy.
- Per-distro build/install dirs: `build/<distro>`, `install/<distro>`.
- `ros2 pkg list` shows both packages.
- `contact_graspnet_ros/core` imports without ROS.
- Acceptance: clean build; no submodule modifications; `git status` in the submodule remains clean.

### FR-2 — Engine loads the model

- Load `config/model.yaml` (not the submodule's checkpoint config).
- Load `checkpoints/contact_graspnet/checkpoints/model.pt`.
- `model.eval()`, parameters frozen, `torch.inference_mode()` around forward.
- Device selection: `cuda`, `cpu`, or `auto`; fail clearly if CUDA is requested but unavailable.
- Seed: set Python/NumPy/Torch seeds; document residual nondeterminism.
- Acceptance: engine loads model on GPU and CPU; missing checkpoint raises a clear error; unit test verifies state-dict keys and eval mode.

### FR-3 — Inference pipeline

- depth + K + optional mask → point cloud(s).
- `z_range` crop.
- Optional local regions around segments.
- Optional contact filtering by segment.
- Model forward.
- Grasp selection with the port's thresholds.
- Output: flat list of typed proposals with pose, score, width, contact point, segment id.
- Acceptance: unit tests for `depth2pc`, regularization counts, selection indices, pose convention; GPU integration test produces non-empty grasps on the bundled test data.

### FR-4 — Topic mode

- Subscribe to depth (`sensor_msgs/Image`), camera info (`sensor_msgs/CameraInfo`) and optional mask (`sensor_msgs/Image`).
- Synchronize with `message_filters.ApproximateTimeSynchronizer`.
- Drop stale frames when inference is busy; count drops.
- Publish:
  - `grasps` (`GraspProposalArray`)
  - `contact_points` (`sensor_msgs/PointCloud2`)
  - `markers` (`visualization_msgs/MarkerArray`)
  - `diagnostics` (`diagnostic_msgs/DiagnosticArray`)
- Preserve sensor timestamps and frame ids.
- Acceptance: launch test with recorded data publishes all outputs; header stamps match input; RViz markers appear.

### FR-5 — Service mode

- `generate_grasps` (`GenerateGrasps`): depth + camera info + optional mask.
- `generate_grasps_from_pointclouds`: full cloud + optional segment cloud.
- Response: `success`, `message`, `GraspProposalArray`.
- Serialize GPU access through the same engine lock.
- If busy beyond timeout, return `success=false` with a clear message.
- Acceptance: service tests with recorded data; concurrent calls do not crash; error paths return failure.

### FR-6 — Configuration

- All pipeline configuration lives in `config/`.
- `config/graspnet.yaml` contains ROS parameters.
- `config/model.yaml` contains model architecture and thresholds.
- `config/rviz/` and `config/zenoh/` contain auxiliary config.
- The launch file resolves config from `GRASPNET_CONFIG_DIR` (set by Pixi/Docker), with a `config_dir` launch argument override.
- Docker mounts `config/` read-only.
- Acceptance: editing a threshold in the mounted `config/graspnet.yaml` and restarting changes behavior without rebuilding the image.

### FR-7 — Python quality and typing

- Public core APIs fully type-annotated (`numpy.typing`, dataclasses, `Protocol` where useful).
- `ruff` and `mypy` (with `ignore_missing_imports` for the submodule) pass.
- Node code typed and small; ROS conversions isolated.
- Acceptance: `pixi run lint` and `pixi run test` pass.

### FR-8 — Pixi environments

- `jazzy` (default): Python 3.12, ROS Jazzy, PyTorch CUDA, build tools.
- `humble`: secondary build target. The current RoboStack Humble channel is not solvable with Python 3.10 for the required Zenoh/launch-testing packages, so the Pixi environment uses Python 3.12 and Fast DDS. Native Humble remains Python 3.10; verify separately.
- `jazzy-cpu` (optional): CPU-only PyTorch for unit tests.
- Committed `pixi.lock`.
- Tasks: `build`, `test`, `lint`, `launch`, `zenohd`, `clean`.
- Acceptance: `pixi run -e jazzy build` and `pixi run -e humble build` succeed in a clean env.

### FR-9 — Docker and Compose

- `docker/Dockerfile`: multi-stage Pixi build + production runtime.
- Default Jazzy image; `PIXI_ENV`/`BASE_IMAGE` build args allow Humble later.
- No TensorFlow, no TF custom ops, no C++ node, no CUDA compiler.
- `docker/compose.contact_graspnet.yaml`: include-able service with GPU reservation, host network, config/checkpoint mounts, env vars, healthcheck.
- `docker-compose.yaml`: includes the service and adds a Zenoh router; runnable standalone.
- Checkpoints are mounted, not baked into the image (license/size).
- `.dockerignore` excludes build artifacts, `.pixi`, `.git`, caches, test data and checkpoint weights.
- Acceptance: `docker compose -f docker-compose.yaml up` starts the node; config mount works; `docker compose config` is valid.

### FR-10 — RMW and Zenoh

- Default `RMW_IMPLEMENTATION=rmw_zenoh_cpp` on Jazzy.
- `rmw_zenohd` router service in the standalone Compose file.
- Code is RMW-agnostic; switching to Fast DDS via env var works.
- Acceptance: topics/services work over Zenoh; router healthcheck passes.

### FR-11 — Observability

- Startup log: checkpoint path, model config path, device, Torch/CUDA versions, ROS distro.
- Per-inference debug log: input shape, num points, num segments, num grasps, latency.
- Diagnostics: device, inference time, grasps, dropped frames, last error.
- Acceptance: diagnostics topic is populated at the configured rate.

### FR-12 — Documentation

- `README.md`: quickstart, prerequisites, submodule init, checkpoint setup, Pixi commands, Docker, topics/services, troubleshooting.
- `docs/inference_contract.md`: input/output types, shapes, frames, pose convention.
- `docs/ros_interfaces.md`: messages, services, topics, QoS.
- `docs/deployment.md`: Docker/Compose, config mounts, Zenoh, integration.
- `docs/architecture.md`: Python node/core boundary and data flow.
- Acceptance: a new developer can run the node from a clean checkout using only the README.

---

## 7. Interfaces

### 7.1 Messages

`msg/GraspProposal.msg`

```text
# Gripper-base pose in the frame of GraspProposalArray.header.
# +Z = approach, +X = finger-closing direction.
geometry_msgs/Pose pose
float32 score
float32 width
geometry_msgs/Point contact_point
int32 segment_id
```

`msg/GraspProposalArray.msg`

```text
std_msgs/Header header
uint32 num_input_points
float32 inference_time_s
GraspProposal[] grasps
```

### 7.2 Services

`srv/GenerateGrasps.srv`

```text
sensor_msgs/Image depth
sensor_msgs/CameraInfo camera_info
sensor_msgs/Image mask
int32 segmap_id
bool local_regions
bool filter_grasps
bool skip_border_objects
float64[2] z_range
uint8 forward_passes
---
bool success
string message
contact_graspnet_msgs/GraspProposalArray grasps
```

`srv/GenerateGraspsFromPointClouds.srv`

```text
sensor_msgs/PointCloud2 cloud
sensor_msgs/PointCloud2 segment
int32 segmap_id
bool local_regions
bool filter_grasps
uint8 forward_passes
---
bool success
string message
contact_graspnet_msgs/GraspProposalArray grasps
```

Rules:

- `segmap_id > 0` is required when a mask/segment is supplied; `0` means all labels in topic mode.
- Empty `mask`/`segment` means no segmentation; `local_regions` and `filter_grasps` must then be false.
- Service responses use the input frame; the node does not transform to `output_frame` for services unless the request is extended later. Document this.

### 7.3 Topics

| Direction | Topic | Type | QoS |
|---|---|---|---|
| Input | `~/depth` | `sensor_msgs/Image` | sensor data |
| Input | `~/camera_info` | `sensor_msgs/CameraInfo` | sensor data |
| Input | `~/mask` | `sensor_msgs/Image` | sensor data |
| Output | `grasps` | `GraspProposalArray` | reliable |
| Output | `contact_points` | `sensor_msgs/PointCloud2` | reliable |
| Output | `markers` | `visualization_msgs/MarkerArray` | reliable |
| Output | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable |

Topic names are parameters/remappable. A namespace such as `/graspnet` is recommended.

### 7.4 Parameters

| Parameter | Type | Default | Notes |
|---|---|---|---|
| `model_config` | string | `model.yaml` | relative to `GRASPNET_CONFIG_DIR` |
| `checkpoint_dir` | string | — | directory containing `model.pt` |
| `device` | string | `cuda` | `cuda`, `cpu`, `auto` |
| `depth_topic` | string | `depth` | relative/absolute |
| `camera_info_topic` | string | `camera_info` | |
| `mask_topic` | string | `mask` | empty disables mask |
| `output_frame` | string | `""` | empty = input frame |
| `sync_slop_s` | double | `0.05` | |
| `z_range` | double[2] | `[0.2, 1.8]` | metres |
| `local_regions` | bool | `true` | requires mask |
| `filter_grasps` | bool | `true` | requires mask |
| `skip_border_objects` | bool | `false` | |
| `segmap_id` | int | `0` | `0` = all labels |
| `max_segments` | int | `4` | safety cap |
| `forward_passes` | int | `1` | dynamic in PyTorch |
| `first_thres` | double | `0.15` | port default |
| `second_thres` | double | `0.15` | port default |
| `num_samples` | int | `200` | |
| `max_farthest_points` | int | `150` | |
| `seed` | int | `0` | |
| `publish_markers` | bool | `true` | |
| `publish_contact_points` | bool | `true` | |
| `input_qos` | string | `sensor_data` | or `default` |
| `output_qos` | string | `reliable` | or `sensor_data` |
| `tf_timeout_s` | double | `0.1` | |
| `service_timeout_s` | double | `5.0` | |
| `diagnostics_rate_hz` | double | `1.0` | |
| `mask_mode` | string | `labels` | or `binary` |
| `depth_scale` | double | `0.001` | for 16UC1 |

Validation at startup:

- `checkpoint_dir/model.pt` exists.
- `model_config` exists and contains required keys.
- `z_range[0] < z_range[1]`.
- `forward_passes >= 1`.
- `local_regions`/`filter_grasps` require `mask_topic` or service mask.
- `segmap_id >= 0`.
- `device` is valid; if `cuda` and no GPU, fail with a clear message.

### 7.5 Config files

`config/graspnet.yaml` — ROS parameters:

```yaml
/**:
  ros__parameters:
    model_config: model.yaml
    checkpoint_dir: checkpoints/contact_graspnet
    device: cuda
    depth_topic: depth
    camera_info_topic: camera_info
    mask_topic: mask
    output_frame: ""
    sync_slop_s: 0.05
    z_range: [0.2, 1.8]
    local_regions: true
    filter_grasps: true
    skip_border_objects: false
    segmap_id: 0
    max_segments: 4
    forward_passes: 1
    first_thres: 0.15
    second_thres: 0.15
    num_samples: 200
    max_farthest_points: 150
    seed: 0
    publish_markers: true
    publish_contact_points: true
    input_qos: sensor_data
    output_qos: reliable
    tf_timeout_s: 0.1
    service_timeout_s: 5.0
    diagnostics_rate_hz: 1.0
    mask_mode: labels
    depth_scale: 0.001
```

`config/model.yaml` — copy of `third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/config.yaml` with a header comment:

```yaml
# Model configuration for Contact-GraspNet PyTorch port.
# Source: third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/config.yaml
# The ROS wrapper treats this file as authoritative for model architecture.
# Runtime thresholds may be overridden by ROS parameters.
```

Precedence:

1. `config/model.yaml` defines architecture and defaults.
2. `config/graspnet.yaml` overrides the explicitly exposed runtime fields.
3. Service request fields override the corresponding node parameters for that call.

### 7.6 Core engine API

`core/types.py`:

```python
FloatArray = npt.NDArray[np.float32]
IntArray = npt.NDArray[np.int32]

@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float; fy: float; cx: float; cy: float
    width: int; height: int

@dataclass(frozen=True)
class InferenceOptions:
    z_range: tuple[float, float] = (0.2, 1.8)
    local_regions: bool = True
    filter_grasps: bool = True
    skip_border_objects: bool = False
    segmap_id: int = 0
    max_segments: int = 4
    forward_passes: int = 1
    first_thres: float = 0.15
    second_thres: float = 0.15
    num_samples: int = 200
    max_farthest_points: int = 150

@dataclass(frozen=True)
class GraspProposal:
    pose: FloatArray          # (4,4)
    score: float
    width: float
    contact_point: FloatArray # (3,)
    segment_id: int

@dataclass(frozen=True)
class GraspResult:
    proposals: tuple[GraspProposal, ...]
    num_input_points: int
    inference_time_s: float
    warnings: tuple[str, ...]
```

`core/engine.py` public methods:

```python
class ContactGraspNetEngine:
    def __init__(self, model_config_path: str, checkpoint_path: str,
                 device: str = "cuda", seed: int = 0) -> None: ...
    def infer(self, depth: FloatArray, intrinsics: CameraIntrinsics,
              mask: IntArray | None, options: InferenceOptions) -> GraspResult: ...
    def infer_pointclouds(self, points: FloatArray, segment: FloatArray | None,
                          options: InferenceOptions) -> GraspResult: ...
    def close(self) -> None: ...
```

The node converts `GraspResult` to ROS messages. No ROS types enter `core/`.

---

## 8. Implementation details

### 8.1 Core engine

`core/config.py`:

- `load_model_config(path) -> dict`.
- `validate_model_config(cfg) -> None`.
- `resolve_model_defaults(cfg) -> ModelDefaults`.
- Uses `yaml.safe_load`.

`core/geometry.py`:

- `depth_to_point_cloud(depth, K) -> FloatArray`.
- `extract_point_clouds(depth, K, mask, options) -> tuple[FloatArray, dict[int, FloatArray]]`.
- `regularize_pc_point_count(pc, npoints, rng)`.
- `preprocess_pc_for_inference(pc, num_points, rng, convert_to_internal_coords=True)`.
- `extract_3d_cam_boxes(full_pc, segments, raw_num_points, rng, min_size=0.2, max_size=0.6)`.
- `select_grasps(contact_pts, scores, max_farthest_points, num_samples, first_thres, second_thres)`.
- `filter_segment(contact_pts, segment_pc, thres)`.
- All functions typed and documented; pure NumPy/SciPy, no ROS.

`core/model.py`:

- `class PatchedContactGraspnet(ContactGraspnet)`:
  - override `build_6d_grasp` with `torch.cross(..., dim=2)`.
- `build_model(cfg, device) -> PatchedContactGraspnet`.
- `load_checkpoint(model, checkpoint_path, device) -> None`:
  - `torch.load(..., map_location=device, weights_only=True)`
  - extract `state["model"]`; fail clearly if missing.
  - `model.load_state_dict(state, strict=True)`.
  - `model.eval()`, `requires_grad_(False)`.

`core/engine.py`:

- `__init__`: load config, build model, load checkpoint, set seed.
- `infer`: validate shapes/dtypes; `extract_point_clouds`; call `predict_scene_grasps`; normalize; return.
- `infer_pointclouds`: use supplied full cloud and segment.
- `predict_scene_grasps`: local regions or full cloud; call `predict_grasps`; filter; normalize keys.
- `predict_grasps`: preprocess, model forward in `torch.inference_mode()`, uncenter, compute widths, select, convert back to OpenCV/ROS optical.
- `close`: release model/torch cache; best effort.

Output normalization:

- `poses = np.asarray(poses).reshape(-1, 4, 4).astype(np.float32)`
- `scores = np.asarray(scores).reshape(-1).astype(np.float32)`
- `widths = np.asarray(widths).reshape(-1).astype(np.float32)`
- `contacts = np.asarray(contacts).reshape(-1, 3).astype(np.float32)`
- `segment_ids` per proposal.
- Sort by score descending.
- Cap total proposals at `num_samples * max_segments` (safety).
- Ignore `-1` keys when segment keys exist.

Seed and determinism:

- Set `random.seed`, `np.random.seed`, `torch.manual_seed`, `torch.cuda.manual_seed_all`.
- Document that GPU ops and FPS sampling may be nondeterministic.
- `torch.use_deterministic_algorithms(True)` optional via `deterministic: true`; warn if it fails.

### 8.2 rclpy node

`node.py`:

- `class ContactGraspNetNode(Node)`.
- Declare/read/validate parameters.
- Build `NodeConfig`.
- Create engine.
- Create publishers, services, subscribers, synchronizer.
- Diagnostics timer.
- `MultiThreadedExecutor` with `MutuallyExclusiveCallbackGroup` for inference callbacks and service callbacks.
- A `threading.Lock` around `engine.infer` ensures one inference at a time.
- Topic callback: `acquire(blocking=False)`; if busy, increment dropped and return.
- Service callback: `acquire(timeout=service_timeout_s)`; if timeout, return busy.
- Publish from the callback thread after inference.
- On shutdown: destroy engine, close; no Python interpreter work.

`conversions.py`:

- `sensor_msgs/Image` → `np.ndarray` depth (`32FC1`) or `16UC1` with `depth_scale`.
- `sensor_msgs/Image` → `np.ndarray` mask (`mono8`, `mono16`, `32SC1`); binary/labels mode.
- `CameraInfo` → `CameraIntrinsics` (P preferred, K fallback; validate dimensions).
- `sensor_msgs/PointCloud2` → `np.ndarray` xyz (`sensor_msgs_py.point_cloud2.read_points`).
- 4×4 pose → `geometry_msgs/Pose` with orthonormalized quaternion.
- TF2 transform of poses/points.
- Markers and contact `PointCloud2`.

`markers.py`:

- Build a `MarkerArray` with one `LINE_LIST` marker:
  - approach axis and two fingers per grasp,
  - color by score,
  - header frame/stamp from the result.
- Optional top-grasp text marker.

`config.py` (ROS-side):

- `NodeConfig` dataclass.
- `from_node(node) -> NodeConfig`.
- Validation and clear errors.
- Resolve relative `model_config`/`checkpoint_dir` against `GRASPNET_CONFIG_DIR`/`CONTACT_GRASPNET_ROOT`.

### 8.3 Threading model

- `MultiThreadedExecutor(num_threads=3)`.
- `MutuallyExclusiveCallbackGroup` for the synchronized depth callback and services.
- `ReentrantCallbackGroup` for diagnostics/other lightweight callbacks if needed.
- `threading.Lock` around the engine.
- Topic mode drops frames when the lock is busy.
- Service mode waits up to `service_timeout_s`; returns busy on timeout.
- Inference runs in the callback thread; PyTorch releases the GIL during tensor ops, so diagnostics/other callbacks can run.
- No unbounded queue; no request pile-up.

### 8.4 TF2 and frames

- `tf2_ros.Buffer` + `TransformListener`.
- If `output_frame` empty or equal to input frame, skip.
- `lookup_transform(output_frame, input_frame, stamp, timeout)`.
- Convert quaternion to a 4×4 matrix (SciPy `Rotation` or NumPy).
- Apply `T_out_in @ T_in_grasp` to each pose; transform contact points.
- If TF fails: log error, diagnostics error, do not publish.
- Orthonormalize rotation before quaternion conversion; normalize quaternion.
- Output header `frame_id = output_frame` (or input frame), `stamp = input stamp`.

### 8.5 Error handling

- Missing checkpoint/config: fail at startup with a clear error; do not continue.
- Invalid image encoding/dimensions: drop frame, log error, diagnostics.
- Engine exception: topic mode logs and drops; service returns `success=false`.
- CUDA OOM: catch, log, return failure; do not crash.
- TF failure: drop frame; do not publish.
- Service timeout: return failure.
- No grasps: publish an empty `GraspProposalArray` (not silence) for topic mode; service returns `success=true` with empty grasps and a message.

### 8.6 Logging and diagnostics

- Startup: ROS distro, Python version, Torch version, CUDA availability, device, checkpoint, model config, thresholds.
- Per inference: debug log with input shape, points, segments, grasps, latency; throttle.
- Diagnostics: device, inference time, num grasps, dropped frames, last error, TF status.
- Python logging goes through the ROS logger where practical; core uses `logging` and does not import rclpy.

---

## 9. Build and environments

### 9.1 Pixi

Top-level channels:

- `https://prefix.dev/conda-forge`
- `https://prefix.dev/robostack-jazzy`
- `https://prefix.dev/robostack-humble`

Features:

- `jazzy`: Python 3.12, `ros-jazzy-ros-base`, `ros-jazzy-rmw-zenoh-cpp`, `ros-jazzy-cv-bridge`, `ros-jazzy-message-filters`, `ros-jazzy-tf2-ros`, `ros-jazzy-tf2-geometry-msgs`, `ros-jazzy-visualization-msgs`, `ros-jazzy-diagnostic-msgs`, `ros-jazzy-rosidl-default-generators`, `ros-jazzy-rosidl-default-runtime`, `ros-jazzy-ament-cmake-auto`, `ros-jazzy-rviz2`, `ros-jazzy-launch-testing`, `ros-jazzy-launch-testing-ament-cmake`, `pytorch`, `numpy`, `scipy`, `pyyaml`, `opencv`, `pillow`, `trimesh`, `colcon-common-extensions`, `cmake`, `ninja`, `compilers`, `pkg-config`, `pytest`, `pytest-cov`, `ruff`, `mypy`.
- `humble`: same with `ros-humble-*`, Python 3.12 on the current RoboStack channel, Fast DDS, PyTorch CUDA 12.9. Native Humble Python 3.10 remains a separate validation target.
- `jazzy-cpu` (optional): CPU-only Torch for unit tests.

Pin Torch and CUDA explicitly; do not use `*`. Commit `pixi.lock`.

Pixi activation:

```bash
export CONTACT_GRASPNET_ROOT="$PIXI_PROJECT_ROOT"
export GRASPNET_CONFIG_DIR="${GRASPNET_CONFIG_DIR:-$PIXI_PROJECT_ROOT/config}"
export PYTHONPATH="$PIXI_PROJECT_ROOT/third_party/contact_graspnet_pytorch:$PYTHONPATH"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
if [ -f "$PIXI_PROJECT_ROOT/install/$ROS_DISTRO/setup.bash" ]; then
  source "$PIXI_PROJECT_ROOT/install/$ROS_DISTRO/setup.bash"
fi
```

Tasks:

- `build`: `colcon build --symlink-install --build-base build/$ROS_DISTRO --install-base install/$ROS_DISTRO --cmake-args -DPython3_EXECUTABLE=$CONDA_PREFIX/bin/python -DPython3_ROOT_DIR=$CONDA_PREFIX -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- `test`: `colcon test` + `pytest src/contact_graspnet_ros/test`
- `lint`: `ruff check`, `mypy`, `python -m compileall`
- `launch`: `ros2 launch contact_graspnet_ros graspnet.launch.py`
- `zenohd`: `ros2 run rmw_zenoh_cpp rmw_zenohd`

The `colcon build` CMake args are still needed for the message package and to ensure Python is the Pixi one.

### 9.2 setup.py / package.xml

`contact_graspnet_ros`:

- `setup.py` with `data_files` for `launch/`, `config/` (optional) and `resource/`.
- `entry_points={'console_scripts': ['graspnet_node = contact_graspnet_ros.node:main']}`.
- `package.xml` dependencies:
  - `rclpy`, `sensor_msgs`, `geometry_msgs`, `std_msgs`, `visualization_msgs`, `diagnostic_msgs`, `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `cv_bridge`, `sensor_msgs_py`, `contact_graspnet_msgs`.
  - `launch`, `launch_ros`, `ament_index_python`.
  - test: `pytest`, `launch_testing`, `launch_testing_ament_cmake`.
- `setup.cfg` with `[develop] script_dir=$base/lib/contact_graspnet_ros`.

`contact_graspnet_msgs`:

- `CMakeLists.txt` with `rosidl_generate_interfaces`.
- `package.xml` with `rosidl_default_generators`, `rosidl_default_runtime`.

### 9.3 Docker

`docker/Dockerfile`:

```dockerfile
ARG PIXI_IMAGE=ghcr.io/prefix-dev/pixi:0.84.1-noble-cuda-12.9
ARG BASE_IMAGE=nvidia/cuda:12.9.1-runtime-ubuntu24.04
ARG PIXI_ENV=jazzy

FROM ${PIXI_IMAGE} AS build
WORKDIR /opt/contact_graspnet_ros
COPY pixi.toml pixi.lock ./
RUN pixi install --locked -e ${PIXI_ENV}
COPY . .
RUN pixi run --locked -e ${PIXI_ENV} build
RUN pixi shell-hook -e ${PIXI_ENV} -s bash > /shell-hook

FROM ${BASE_IMAGE}
WORKDIR /opt/contact_graspnet_ros
# install runtime OS deps (libgl, etc.)
# copy .pixi/envs/${PIXI_ENV}, install/${ROS_DISTRO}, src, config, third_party source
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "contact_graspnet_ros", "graspnet.launch.py"]
```

Notes:

- No TensorFlow, no tf_ops, no CUDA compiler.
- `git submodule update --init --recursive` before `docker build`.
- `.dockerignore` excludes `.git`, `.pixi`, `build`, `install`, `log`, `__pycache__`, `*.pyc`, `results`, `third_party/**/checkpoints` (weights mounted), test data.
- Do not bake `model.pt`; mount it.
- The production stage must preserve the same pixi env path as the build stage (`/opt/contact_graspnet_ros/.pixi/envs/...`) because `shell-hook` bakes absolute paths.

`docker/entrypoint.sh`:

```bash
#!/bin/bash
set -e
source /shell-hook
source /opt/contact_graspnet_ros/install/${ROS_DISTRO}/setup.bash
exec "$@"
```

### 9.4 Compose

`docker/compose.contact_graspnet.yaml` (include-able):

```yaml
services:
  contact_graspnet:
    image: ${GRASPNET_IMAGE:-contact_graspnet_ros:jazzy}
    container_name: contact_graspnet
    network_mode: host
    environment:
      ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-0}
      RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}
      CONTACT_GRASPNET_ROOT: /opt/contact_graspnet_ros
      GRASPNET_CONFIG_DIR: /opt/contact_graspnet_ros/config
    volumes:
      - ${GRASPNET_CONFIG_DIR:-./config}:/opt/contact_graspnet_ros/config:ro
      - ${GRASPNET_CHECKPOINT_DIR:-./third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet}:/opt/contact_graspnet_ros/checkpoints:ro
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    healthcheck:
      test: ["CMD-SHELL", "ros2 node list | grep -q contact_graspnet"]
      interval: 10s
      timeout: 5s
      start_period: 120s
      retries: 6
    restart: unless-stopped
```

`docker-compose.yaml` (root):

```yaml
include:
  - path: docker/compose.contact_graspnet.yaml

services:
  contact_graspnet:
    build:
      context: .
      dockerfile: docker/Dockerfile
      args:
        PIXI_ENV: jazzy
    depends_on:
      zenoh-router:
        condition: service_healthy

  zenoh-router:
    image: ${GRASPNET_IMAGE:-contact_graspnet_ros:jazzy}
    network_mode: host
    command: ["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"]
    healthcheck:
      test: ["CMD-SHELL", "pgrep -x rmw_zenohd"]
      interval: 2s
      timeout: 2s
      retries: 30

  graspnet-echo:
    image: ${GRASPNET_IMAGE:-contact_graspnet_ros:jazzy}
    network_mode: host
    profiles: ["demo"]
    command: ["ros2", "topic", "echo", "/graspnet/grasps", "--once"]
```

Requirements:

- Compose v2.20+ for `include`.
- Host networking recommended for DDS/Zenoh on Linux.
- If the main integration compose already runs a Zenoh router, remove the router service from the standalone file.
- If the pipeline uses Fast DDS, set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.
- Checkpoints are mounted read-only; no rebuild for config or weight changes.

---

## 10. Testing

### 10.1 Core unit tests (no ROS, CPU)

- `test_geometry.py`:
  - `depth_to_point_cloud` with a known depth/K.
  - z-range cropping.
  - segmentation extraction, binary vs labels.
  - `regularize_pc_point_count` count and dtype.
  - `select_grasps` threshold/FPS behavior, including the single-index case.
  - pose convention: known base/approach/contact/width → expected 4×4.
- `test_config.py`:
  - YAML load, missing keys, invalid thresholds.
- `test_engine_smoke.py`:
  - Build model on CPU with a reduced config (small `raw_num_points`/`npoint`) to keep it fast.
  - Load state dict (if feasible on CPU) or test with a random small model.
  - Run one forward pass; assert shapes and finite values.
  - Mark full 20k CPU inference as slow/nightly.

### 10.2 ROS integration tests

- Launch node with a recorded `.npz` replayed as ROS messages.
- Publish depth/camera_info/mask at 5–10 Hz.
- Assert `grasps` publishes with the input stamp/frame.
- Call both services; assert success and non-empty results for a known scene.
- Test no-mask mode, empty results, invalid encoding, TF failure.
- Use `launch_testing` and a fixture that skips if no checkpoint/GPU (unless `GRASPNET_TEST_GPU=1`).

### 10.3 Docker tests

- `docker compose -f docker-compose.yaml config`.
- Build the image and run `ros2 node list` / `ros2 topic list`.
- Mount a test config, restart, verify a threshold change takes effect.
- Verify checkpoints are not inside the image.

### 10.4 Static analysis

- Python: `ruff check`, `ruff format --check`, `mypy` on core/public modules.
- YAML: `yamllint` optional.
- Docker: `hadolint` optional.

### 10.5 CI

- Jazzy: Pixi install, build, core tests, ROS tests, lint.
- Humble: build + core tests (no GPU).
- Docker build (no run).
- GPU integration tests on a self-hosted runner.

---

## 11. Milestones and deliverables

### M0 — Engine feasibility (1–2 days)

- Add `src/contact_graspnet_ros` skeleton and `core/` modules.
- Load `model.pt` on CPU and GPU; call `model.eval()`.
- Run one forward pass on the bundled test scene.
- Measure latency and GPU memory.
- Gate: if the port is unusable or too slow, revisit backend choice before building the node.

### M1 — Core engine + tests (3–4 days)

- `types.py`, `config.py`, `geometry.py`, `model.py`, `engine.py`.
- Typed public API, unit tests, smoke test.
- Document the inference contract.
- Acceptance: engine runs outside ROS; no submodule edits; no pyrender/mayavi imports.

### M2 — Messages + topic mode (3–4 days)

- `contact_graspnet_msgs` package.
- `rclpy` node, parameter parsing, conversions.
- Subscriptions, synchronizer, publishers.
- TF2 output frame.
- Markers and diagnostics.
- Acceptance: recorded depth/mask produces `GraspProposalArray`, contacts, markers; stamps/frames correct.

### M3 — Services + robustness (2–3 days)

- `GenerateGrasps` and `GenerateGraspsFromPointClouds`.
- Busy/timeout/error semantics.
- Parameter validation, startup failure modes.
- Acceptance: service tests; concurrent calls serialize; no crashes.

### M4 — Pixi both distros (2–3 days)

- `pixi.toml`, lock, tasks, activation.
- Jazzy build/run; Humble build (secondary).
- Zenoh router task.
- Acceptance: clean Jazzy build/run; Humble build passes.

### M5 — Docker and Compose (2–3 days)

- `docker/Dockerfile`, entrypoint, `.dockerignore`.
- `docker/compose.contact_graspnet.yaml`, root `docker-compose.yaml`.
- Config/checkpoint mounts, GPU reservation, healthcheck.
- Acceptance: standalone Compose starts; include-able file works; config edit without rebuild.

### M6 — Tests, docs, release (3–4 days)

- Integration tests, CI, README, docs.
- Benchmark table (latency, GPU memory, CPU fallback).
- Acceptance: definition of done below.

---

## 12. Risks and mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| PyTorch port quality/perf | Poor grasps or slow inference | M0 benchmark/parity; fallback to TF backend or optimize PointNet2 |
| Pure-torch FPS/ball query slow | Latency budget missed | Benchmark; consider compiled PointNet2 extension or reduced points; not v1 |
| Python/Torch and ROS conda conflicts | Environment cannot solve | Pin versions; RoboStack + conda-forge; fallback to pip torch in a separate env if needed |
| Submodule update changes state-dict keys | Model fails to load | Pin submodule SHA; validate keys; document upgrade procedure |
| Checkpoint license/size | Cannot redistribute | Mount weights; do not bake; include NOTICE; verify NVIDIA license |
| Humble Torch/CUDA availability | No Humble build | Jazzy first; verify channel; fallback to CPU or later Humble patch |
| Zenoh router/discovery | No communication | Router service + healthcheck; host network; document RMW switch |
| `model.eval()` omission in submodule | Noisy inference | Explicitly call `eval()` and `inference_mode()` in our engine |
| `torch.cross` default dim | Warning/error in future Torch | Subclass override with `dim=2` |
| Service concurrency/latency | Timeouts | Single engine lock; busy response; no unbounded queue; document |
| GPU memory growth | OOM with multiple segments | `max_segments`, one inference at a time, `torch.cuda.empty_cache()` on close |

---

## 13. Definition of done

- `colcon build` succeeds on Jazzy; Humble build succeeds or is documented as blocked.
- `pixi run -e jazzy test` passes; `pixi run -e jazzy lint` passes.
- Core unit tests pass on CPU.
- GPU integration test on the bundled scene produces non-empty grasps with correct stamps/frames.
- Topic mode publishes `GraspProposalArray`, `PointCloud2`, `MarkerArray`, diagnostics.
- Both services work and return typed responses.
- `config/` is the only place to change pipeline configuration; Docker mounts it read-only.
- `docker compose -f docker-compose.yaml up` starts the node and Zenoh router.
- `docker/compose.contact_graspnet.yaml` can be included in a parent Compose file.
- Checkpoints are mounted, not baked into the image.
- README and docs allow a clean-checkout run.
- No changes inside `third_party/contact_graspnet_pytorch`.

---

## 14. Open questions / decisions

1. **Checkpoint distribution:** mount the submodule's `model.pt` (recommended) vs. copy into the image. Check the NVIDIA license.
2. **Default thresholds:** use the port's `0.15/0.15` (recommended) vs. the original TF `0.23/0.19`.
3. **Lifecycle node:** plain `rclpy.Node` in v1 (recommended) vs. `rclpy.lifecycle.LifecycleNode`.
4. **MoveIt bridge:** out of scope for v1; optional separate adapter later.
5. **HTTP bridge for Cap-X:** optional separate package later; it can import `contact_graspnet_ros.core.engine`.
6. **Humble priority:** build target only; full runtime parity is a follow-up.
7. **CPU CI:** decide whether to support a full CPU smoke test or only model construction/loading due to FPS slowness.

---

## Appendix A — Upstream port caveats

- The PyTorch port is unofficial; `model.pt` is a retrained checkpoint, not the official NVIDIA weights. Validate quality on test scenes.
- The port's `GraspEstimator` does not call `model.eval()`; our engine must.
- The port's `predict_scene_grasps_from_depth_K_and_2d_seg()` is broken (passes `sess`); do not use.
- The port's `build_network()`/`load_weights()` are dead TF remnants; do not call.
- The port's `data.py` imports pyrender; our core avoids it.
- The port's `utils.build_6d_grasp()` uses `torch.cross` without `dim`; our model subclass overrides it.
- The port's `select_grasps()` has the `np.any([0])` bug; our geometry copy fixes it.
- The port returns squeezed one-element arrays; our engine normalizes.
- The port retains `-1` keys in some filter paths; our engine normalizes.
- The port's config uses `first_thres=second_thres=0.15`; document this.

## Appendix B — Implementation file checklist

### Root

- [ ] `pixi.toml`, `pixi.lock`
- [ ] `README.md`
- [ ] `.dockerignore`, `.gitignore` updates
- [ ] `config/graspnet.yaml`, `config/model.yaml`
- [ ] `config/rviz/graspnet.rviz`
- [ ] `config/zenoh/` (optional)
- [ ] `docs/architecture.md`, `docs/inference_contract.md`, `docs/ros_interfaces.md`, `docs/deployment.md`
- [ ] `third_party/NOTICE`

### `src/contact_graspnet_msgs`

- [ ] `msg/GraspProposal.msg`
- [ ] `msg/GraspProposalArray.msg`
- [ ] `srv/GenerateGrasps.srv`
- [ ] `srv/GenerateGraspsFromPointClouds.srv`
- [ ] `CMakeLists.txt`, `package.xml`

### `src/contact_graspnet_ros`

- [ ] `contact_graspnet_ros/__init__.py`
- [ ] `contact_graspnet_ros/core/__init__.py`
- [ ] `contact_graspnet_ros/core/types.py`, `config.py`, `geometry.py`, `model.py`, `engine.py`
- [ ] `contact_graspnet_ros/node.py`, `conversions.py`, `markers.py`, `config.py`
- [ ] `launch/graspnet.launch.py`, `launch/graspnet_zenoh.launch.py`
- [ ] `test/test_geometry.py`, `test/test_config.py`, `test/test_engine_smoke.py`, `test/test_ros_integration.py`
- [ ] `package.xml`, `setup.py`, `setup.cfg`
- [ ] `resource/contact_graspnet_ros`

### Docker

- [ ] `docker/Dockerfile`
- [ ] `docker/entrypoint.sh`
- [ ] `docker/compose.contact_graspnet.yaml`
- [ ] `docker-compose.yaml`

### Tests / CI

- [ ] `launch_testing` integration test
- [ ] CPU core tests
- [ ] GPU integration test marker
- [ ] CI workflow (Jazzy, Humble, Docker build)
