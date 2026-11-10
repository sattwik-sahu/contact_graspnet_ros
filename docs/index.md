# Contact-GraspNet ROS 2

## What it does

This repository wraps the PyTorch implementation of
[Contact-GraspNet](https://github.com/elchun/contact_graspnet_pytorch) in a ROS 2
node.

```text
generate_grasps service request
  depth + camera_info + optional mask
        │
        ▼
ROS 2 service node (rclpy)
        │
        ▼
Contact-GraspNet engine (PyTorch, CUDA)
        │
        ▼
GraspProposalArray response
```

The wrapper is split into a small message package and a Python ROS package with
a **ROS-free, typed core**. The vendored model is treated as a read-only
submodule; all preprocessing, inference orchestration and fixes live in this
project.

## Highlights

- 6-DoF grasp proposals with pose, score, gripper width and contact point.
- A single `generate_grasps` service for on-demand inference.
- Typed request/response interface; no sensor-topic subscriptions or output
  streams.
- Single `config/` folder for ROS, model and deployment configuration.
- Pixi environment, Docker image and include-able Compose service.
- Jazzy-first, Humble as a secondary build target.

## Documentation

| Start here | Reference | Project |
|---|---|---|
| [Getting Started](getting-started.md) | [Architecture](architecture.md) | [Contributing](contributing.md) |
| [Configuration](configuration.md) | [ROS Interfaces](ros-interfaces.md) | [License](license.md) |
| [Deployment](deployment.md) | [Inference Contract](inference-contract.md) | [Design / PRD](design.md) |

Source: [sattwik-sahu/contact_graspnet_ros](https://github.com/sattwik-sahu/contact_graspnet_ros)
