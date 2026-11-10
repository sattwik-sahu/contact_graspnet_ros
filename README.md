# Contact-GraspNet ROS 2 wrapper

ROS 2 wrapper around the vendored PyTorch implementation of
[Contact-GraspNet](https://github.com/elchun/contact_graspnet_pytorch).
The wrapper accepts a depth image, camera intrinsics and an optional
segmentation mask, and publishes 6-DoF grasp proposals.

## Packages

| Package | Type | Purpose |
|---|---|---|
| `contact_graspnet_msgs` | `ament_cmake` | Grasp messages and services |
| `contact_graspnet_ros` | `ament_python` | ROS 2 node + ROS-free typed inference engine |

The inference engine lives under `contact_graspnet_ros.core` and does not import
ROS. The `rclpy` node is `contact_graspnet_ros.node`.

## Prerequisites

- Linux x86_64
- Pixi (for host development)
- NVIDIA driver + GPU (CPU inference is possible but slow)
- Docker + NVIDIA Container Toolkit (for the container workflow)
- Git submodules initialized:

```bash
git submodule update --init --recursive
```

The checkpoint is expected at:

```text
third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt
```

The submodule is read-only. Do not edit it.

## Host quickstart (Jazzy)

```bash
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy test

# Terminal 1: Zenoh router
pixi run -e jazzy zenohd

# Terminal 2: node
pixi run -e jazzy launch
```

Alternatively, `pixi run -e jazzy ros2 launch contact_graspnet_ros graspnet_zenoh.launch.py`
starts the router and node together.

`pixi install` downloads ROS 2 Jazzy, PyTorch CUDA and the runtime dependencies.
The first install is large. If your home filesystem is small, point the Pixi
cache at a larger disk:

```bash
export PIXI_CACHE_DIR=/path/to/large/disk/pixi-cache
pixi install -e jazzy
```

To validate the model and checkpoint without ROS:

```bash
pixi run -e jazzy python scripts/run_inference.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --device cuda
```

The launch file defaults to:

- namespace: `graspnet`
- depth: `graspnet/depth`
- camera info: `graspnet/camera_info`
- mask: `graspnet/mask`
- config: `config/`

Remap or override topics in the launch file or via a parameter file.

### Running without a mask

Set all of the following in `config/graspnet.yaml`:

```yaml
mask_topic: ""
local_regions: false
filter_grasps: false
```

Otherwise the node waits for the mask topic and produces no output.

## Configuration

All pipeline configuration is in `config/`:

- `config/graspnet.yaml` — ROS parameters.
- `config/model.yaml` — model architecture and default thresholds.
- `config/rviz/graspnet.rviz` — RViz layout.

Docker mounts `config/` read-only, so changing a threshold only requires a
container restart, not a rebuild.

Path overrides:

- `GRASPNET_CONFIG_DIR` — directory containing `graspnet.yaml` and `model.yaml`.
- `GRASPNET_CHECKPOINT_DIR` — directory containing `model.pt`.
- `CONTACT_GRASPNET_ROOT` — repository root used to locate the vendored port.

Important defaults:

- `first_thres` / `second_thres`: `0.15` (PyTorch port checkpoint defaults).
- `z_range`: `[0.2, 1.8]` metres.
- `local_regions` / `filter_grasps`: `true` (require a mask).
- `segmap_id`: `0` means all positive labels; use a positive id for a target.

## ROS interfaces

### Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `depth` | `sensor_msgs/Image` (`32FC1` or `16UC1`) |
| Input | `camera_info` | `sensor_msgs/CameraInfo` |
| Input | `mask` | `sensor_msgs/Image` (`mono8`, `mono16`, `32SC1`) |
| Output | `grasps` | `contact_graspnet_msgs/GraspProposalArray` |
| Output | `contact_points` | `sensor_msgs/PointCloud2` |
| Output | `markers` | `visualization_msgs/MarkerArray` |
| Output | `diagnostics` | `diagnostic_msgs/DiagnosticArray` |

### Services

- `generate_grasps` — depth + camera info + optional mask.
- `generate_grasps_from_pointclouds` — full cloud + optional segment cloud.

Both services return `success`, `message` and a `GraspProposalArray`.

Relative topic/service names resolve under the launch namespace (`graspnet` by
default), e.g. `/graspnet/depth`, `/graspnet/grasps` and
`/graspnet/generate_grasps`.

## Frames and pose convention

- Inputs are expected in the camera optical frame (REP 103: x right, y down,
  z forward).
- Outputs use the input frame by default. Set `output_frame` to transform with
  TF2 at the sensor timestamp.
- The 4x4/pose is the **gripper base frame**, not the contact point or TCP:
  - `+Z` = approach direction
  - `+X` = finger-closing direction
  - translation = contact + width/2 * closing - 0.1034 m * approach
- `width` is the finger opening in metres.

## Docker

Initialize submodules before building:

```bash
git submodule update --init --recursive
docker compose build
docker compose up
```

The root `docker-compose.yaml` includes
`docker/compose.contact_graspnet.yaml` and adds a Zenoh router. This requires
Docker Compose v2.24+ for `include.project_directory`. The include-able file
can be reused from a larger Compose project:

```yaml
include:
  - path: docker/compose.contact_graspnet.yaml
```

The checkpoint is mounted, not baked into the image. Override the mount with:

```bash
GRASPNET_CHECKPOINT_DIR=/absolute/path/to/checkpoints docker compose up
```

## Humble note

Jazzy is the first-class target. The Pixi Humble environment is a secondary
build target. RoboStack's current Humble channel does not provide a solvable
`rmw-zenoh-cpp` build for Python 3.10, so the Pixi Humble environment uses
Python 3.12 and Fast DDS. Native Humble remains Python 3.10; verify package
availability before relying on it.

## Troubleshooting

- **`Could not locate third_party/contact_graspnet_pytorch`** — run
  `git submodule update --init --recursive`, or set `CONTACT_GRASPNET_ROOT`.
- **`checkpoint not found`** — verify `model.pt` exists at the path above.
- **CUDA not available** — set `device: cpu` in `config/graspnet.yaml` for a
  slow fallback, or fix the NVIDIA driver/container runtime.
- **No grasps** — provide a mask and set `segmap_id`; check `z_range`; lower
  `first_thres`/`second_thres`.
- **Wrong frame** — check `camera_info.header.frame_id` and the TF tree.

## License

The wrapper code is Apache-2.0. The vendored Contact-GraspNet PyTorch port and
weights are covered by the upstream NVIDIA license. See
`third_party/contact_graspnet_pytorch/License.pdf` and `third_party/NOTICE`.
Do not redistribute model weights without checking the license.
