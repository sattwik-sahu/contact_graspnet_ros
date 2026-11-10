---
layout: page
title: Getting Started
description: Install, build and run the Contact-GraspNet ROS 2 wrapper.
permalink: /getting-started/
---

## Prerequisites

- Linux x86_64
- [Pixi](https://pixi.sh/)
- NVIDIA driver and a CUDA-capable GPU (CPU inference works but is slow)
- Docker with the NVIDIA Container Toolkit, for the container workflow

## 1. Clone with submodules

```bash
git clone --recurse-submodules git@github.com:sattwik-sahu/contact_graspnet_ros.git
cd contact_graspnet_ros
```

If you already cloned without submodules:

```bash
git submodule update --init --recursive
```

The model is expected at:

```text
third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt
```

The submodule is read-only. Do not edit it.

## 2. Install the Pixi environment

```bash
pixi install -e jazzy
```

The first install is large (ROS 2, PyTorch CUDA, RViz). If your home filesystem
is small, put the Pixi cache on a larger disk:

```bash
export PIXI_CACHE_DIR=/path/to/large/disk/pixi-cache
pixi install -e jazzy
```

## 3. Build and test

```bash
pixi run -e jazzy build
pixi run -e jazzy test
```

## 4. Run

The default RMW on Jazzy is Zenoh. Start a router in one terminal:

```bash
pixi run -e jazzy zenohd
```

Start the node in another:

```bash
pixi run -e jazzy launch
```

Or start the router and node together:

```bash
pixi run -e jazzy ros2 launch contact_graspnet_ros graspnet_zenoh.launch.py
```

The wrapper is service-only. The service is:

```text
/graspnet/generate_grasps
```

Call it with the provided helper:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1
```

See [ROS Interfaces]({{ '/ros-interfaces/' | relative_url }}) for the request
and response schema.

## 5. Validate without ROS

```bash
pixi run -e jazzy python scripts/run_inference.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --device cuda --segmap-id 1 --z-min 0.2 --z-max 2.0
```

This should print the number of grasps, input points and inference time.

## 6. Docker

```bash
docker compose build
docker compose up
```

The root `docker-compose.yaml` includes
`docker/compose.contact_graspnet.yaml` and starts a Zenoh router. The checkpoint
is mounted read-only; it is not baked into the image.

See [Deployment]({{ '/deployment/' | relative_url }}) for Compose integration
and configuration mounts.

## Next steps

- Tune parameters in [Configuration]({{ '/configuration/' | relative_url }}).
- Learn the pose and frame conventions in
  [Inference Contract]({{ '/inference-contract/' | relative_url }}).
- Read [Contributing]({{ '/contributing/' | relative_url }}) before opening a
  pull request.
