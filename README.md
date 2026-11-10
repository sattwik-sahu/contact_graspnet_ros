# Contact-GraspNet ROS 2

[![Documentation](https://img.shields.io/badge/docs-online-6750A4?style=flat-square)](https://sattwik-sahu.github.io/contact_graspnet_ros/)
[![Docs build](https://github.com/sattwik-sahu/contact_graspnet_ros/actions/workflows/docs.yml/badge.svg?branch=main)](https://github.com/sattwik-sahu/contact_graspnet_ros/actions/workflows/docs.yml)
[![Docker build](https://github.com/sattwik-sahu/contact_graspnet_ros/actions/workflows/docker.yml/badge.svg?branch=main)](https://github.com/sattwik-sahu/contact_graspnet_ros/actions/workflows/docker.yml)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E?style=flat-square)](https://docs.ros.org/en/jazzy/)
[![Python 3.12](https://img.shields.io/badge/python-3.12-3776AB?style=flat-square)](https://www.python.org/)

A modular, Dockerized ROS 2 wrapper for
[Contact-GraspNet](https://github.com/elchun/contact_graspnet_pytorch).

**Documentation:** https://sattwik-sahu.github.io/contact_graspnet_ros/

```text
GenerateGrasps service request
  depth + camera_info + optional mask
        │
        ▼
rclpy service  ──►  typed PyTorch engine  ──►  GraspProposalArray
```

The wrapper is **service-only**; there are no sensor-topic subscriptions or
output streams.

## Quick start

```bash
git clone --recurse-submodules git@github.com:sattwik-sahu/contact_graspnet_ros.git
cd contact_graspnet_ros
pixi install -e jazzy
pixi run -e jazzy build
pixi run -e jazzy test
```

Start the Zenoh router and node:

```bash
pixi run -e jazzy zenohd    # terminal 1
pixi run -e jazzy launch    # terminal 2
```

Call the service:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1
```

## Packages

| Package | Purpose |
|---|---|
| `contact_graspnet_msgs` | Grasp messages and the `GenerateGrasps` service |
| `contact_graspnet_ros` | rclpy node + ROS-free typed PyTorch engine |

Configuration is in [`config/`](config/), Docker assets in
[`docker/`](docker/), and the documentation source in [`docs/`](docs/).

## Documentation

- [Getting Started](https://sattwik-sahu.github.io/contact_graspnet_ros/getting-started/)
- [Configuration](https://sattwik-sahu.github.io/contact_graspnet_ros/configuration/)
- [ROS Interfaces](https://sattwik-sahu.github.io/contact_graspnet_ros/ros-interfaces/)
- [Deployment](https://sattwik-sahu.github.io/contact_graspnet_ros/deployment/)
- [Contributing](https://sattwik-sahu.github.io/contact_graspnet_ros/contributing/)

## License

Wrapper code: Apache-2.0. The vendored PyTorch port and weights retain the
upstream NVIDIA license; see
[`third_party/NOTICE`](third_party/NOTICE) and
[`docs/license.md`](docs/license.md).
