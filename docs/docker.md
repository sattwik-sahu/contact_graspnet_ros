# Docker and Docker Compose

This guide covers building and running the wrapper in containers, and how to
include it in a larger Compose project.

## Overview

There are three relevant files:

| File | Purpose |
|---|---|
| `docker/Dockerfile` | Multi-stage image: builds the Pixi environment and colcon install space, then copies the runtime into an NVIDIA CUDA runtime base |
| `docker/compose.contact_graspnet.yaml` | Include-able service definition for `contact_graspnet` |
| `docker-compose.yaml` | Root Compose project: includes the service above and adds a Zenoh router |

The image contains the Pixi environment, the colcon install space, the wrapper
source and the vendored port source. It **does not** contain the model weights;
they are mounted read-only at run time.

Both services in the root project use:

```yaml
network_mode: host
ipc: host
```

See [Networking and IPC](#networking-and-ipc) for why.

## Prerequisites

- Linux (host networking is a Linux-only Docker feature; Docker Desktop on
  macOS/Windows does not support `network_mode: host`).
- Docker Engine with Compose v2 (`docker compose`, not the legacy `docker-compose`).
- An NVIDIA GPU and the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
  for GPU inference.
- Git submodules initialized:

  ```bash
  git submodule update --init --recursive
  ```

## Quick start

```bash
git submodule update --init --recursive
docker compose build
docker compose up
```

`docker compose up` starts the `zenoh-router` first and waits for its health
check, then starts `contact_graspnet`. The node's health check waits up to
180 s for the engine to load before checking `ros2 node list`.

Verify:

```bash
docker compose ps
docker compose logs -f contact_graspnet
```

The service is available at `/graspnet/generate_grasps`.

## Services

| Service | Image | Network | IPC | Role |
|---|---|---|---|---|
| `contact_graspnet` | `${GRASPNET_IMAGE:-contact_graspnet_ros:jazzy}` | host | host | ROS 2 service node + PyTorch engine |
| `zenoh-router` | same image | host | host | Zenoh router for RMW discovery |

The router service is only needed when nothing else on the host already runs
one.

## Environment variables

| Variable | Default | Used by | Purpose |
|---|---|---|---|
| `GRASPNET_IMAGE` | `contact_graspnet_ros:jazzy` | both | Image tag |
| `ROS_DOMAIN_ID` | `0` | both | ROS domain |
| `RMW_IMPLEMENTATION` | `rmw_zenoh_cpp` | `contact_graspnet` | RMW selection |
| `GRASPNET_CONFIG_DIR` | `./config` | `contact_graspnet` | Host config directory |
| `GRASPNET_CHECKPOINT_DIR` | `./third_party/.../checkpoints` | `contact_graspnet` | Host checkpoint directory |

Example `.env` file:

```dotenv
GRASPNET_IMAGE=contact_graspnet_ros:jazzy
ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_zenoh_cpp
GRASPNET_CONFIG_DIR=./config
GRASPNET_CHECKPOINT_DIR=./third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints
```

## Mounts

| Host | Container | Mode | Purpose |
|---|---|---|---|
| `${GRASPNET_CONFIG_DIR:-./config}` | `/opt/contact_graspnet_ros/config` | read-only | `graspnet.yaml` and `model.yaml` |
| `${GRASPNET_CHECKPOINT_DIR:-...}` | `/opt/contact_graspnet_ros/checkpoints` | read-only | `model.pt` |

Because the config mount is read-only, a parameter change only needs a
container restart, not an image rebuild. The frame/tool parameters
(`target_frame`, `tcp_offset`, `tcp_rpy`, `tf_timeout_s`) are read at node
startup, so restart the service after editing them.

## GPU

Compose requests one NVIDIA GPU:

```yaml
deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: 1
          capabilities: [gpu]
```

Verify the host setup with `nvidia-smi` and a test container. For CPU-only
inference, remove the `deploy` block and set `device: cpu` in
`config/graspnet.yaml`.

## Networking and IPC

### `network_mode: host`

The container shares the host's network namespace. No ports need to be
published, and ROS 2 / Zenoh discovery works the same as it does natively on
the host. This avoids the common "container sees no other ROS nodes" problem.

### `ipc: host`

The container shares the host's IPC namespace, including `/dev/shm`. This
matters for two reasons:

- ROS 2 transports that use shared memory can exchange large messages without
  the default 64 MB container `/dev/shm` limit (depth images and point clouds
  hit this quickly).
- PyTorch work that uses shared memory (dataloader workers, tensor sharing)
  behaves the same as on the host.

Add both settings to any additional service you create that talks to this
pipeline (for example a segmentation node or a MoveIt planning container).

### Using an existing router

If a Zenoh router already runs on the host or in another Compose project,
remove the `zenoh-router` service. Otherwise two routers on the same host can
compete. For Fast DDS, set `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` and remove the
router.

## Calling the service

From another ROS 2 node on the host (with the same RMW and domain):

```python
from contact_graspnet_msgs.srv import GenerateGrasps
# request.target_frame = "base_link"
```

From inside the container:

```bash
docker compose exec contact_graspnet ros2 service list | grep generate_grasps
docker compose exec contact_graspnet ros2 node list
```

To run the bundled client inside the container, mount a scene directory first
(the sample `.npy` files are excluded from the image by `.dockerignore`):

```yaml
# add to docker/compose.contact_graspnet.yaml for local use
volumes:
  - ./scenes:/scenes:ro
```

```bash
docker compose exec contact_graspnet python scripts/call_grasp_service.py \
  --input /scenes/7.npy \
  --segmap-id 1 \
  --target-frame base_link
```

The container entrypoint sources the Pixi shell hook and the colcon install
space before running the command, so ROS Python modules are importable.

## Including in another Compose project

The root file uses Compose `include`:

```yaml
include:
  - path: docker/compose.contact_graspnet.yaml
    project_directory: .
services:
  contact_graspnet:
    depends_on:
      my_zenoh_router:
        condition: service_healthy
```

The included file defines only `contact_graspnet` (already with host network
and IPC). A parent project can add `depends_on`, extra mounts, or its own
router. See [Grasp Frames and Downstream Planning](grasp-frames.md) for the
frame and tool settings consumed by the service.

## Common commands

```bash
docker compose build                     # rebuild the image
docker compose up -d                     # start detached
docker compose logs -f contact_graspnet  # follow node logs
docker compose ps                        # status + health
docker compose exec contact_graspnet bash
docker compose down                      # stop and remove containers
docker compose config                    # validate and view the merged config
```

## Build details

The Dockerfile has two stages:

1. `build` — installs the locked Pixi environment and runs `pixi run build`,
   producing `/shell-hook` and the colcon `install/` space.
2. `production` — copies the environment, install space, source, config and
   scripts into an NVIDIA CUDA runtime image with the GUI libraries needed by
   OpenCV.

`CONDA_OVERRIDE_CUDA=12.9` lets the build machine install the locked GPU
environment without a GPU present. `.dockerignore` excludes caches, build
directories, docs, site output and the checkpoint weights.

CI (`Dockerfile`-related pushes on `main` and `dev/**`) verifies that the image
builds; it does not push an image.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `could not select device driver ... nvidia` | NVIDIA Container Toolkit missing | Install it and restart Docker |
| `service 'contact_graspnet' didn't reach healthy` | Model load failed or config path wrong | Check `docker compose logs`; verify mounts |
| Other ROS nodes invisible | Router not running, domain/RMW mismatch | Check `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`, router logs |
| `cannot transform ... camera_optical -> base_link` | Missing TF between camera and target frame | Check `ros2 run tf2_ros tf2_echo base_link camera_optical` |
| Shared memory / bus errors on large images | `/dev/shm` too small | Confirm `ipc: host` is present on the service |
| `network_mode: host` ignored on macOS/Windows | Feature unsupported outside Linux | Run on a Linux host or VM |

## Related pages

- [Getting Started](getting-started.md)
- [Configuration](configuration.md)
- [Deployment](deployment.md)
- [ROS Interfaces](ros-interfaces.md)
- [Grasp Frames and Downstream Planning](grasp-frames.md)
