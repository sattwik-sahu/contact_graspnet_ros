---
layout: page
title: Troubleshooting
description: Common setup, GPU and runtime problems.
permalink: /troubleshooting/
---

## `Could not locate third_party/contact_graspnet_pytorch`

Initialize the submodule:

```bash
git submodule update --init --recursive
```

or set:

```bash
export CONTACT_GRASPNET_ROOT="$PWD"
```

## `checkpoint not found`

Verify:

```text
third_party/contact_graspnet_pytorch/checkpoints/contact_graspnet/checkpoints/model.pt
```

For Docker, check `GRASPNET_CHECKPOINT_DIR` and the bind mount in
`docker/compose.contact_graspnet.yaml`.

## CUDA is not available

Confirm the driver and the container runtime:

```bash
nvidia-smi
pixi run -e jazzy python -c "import torch; print(torch.cuda.is_available())"
```

For a quick slow fallback, set `device: cpu` in `config/graspnet.yaml`.

## Service returns no grasps

- Does the request include a mask? If not, set `local_regions: false` and
  `filter_grasps: false` in the `GenerateGrasps` request.
- Is `segmap_id` correct? Use a positive id for a target object.
- Is the object inside `z_range`?
- Lower `first_thres` / `second_thres` for more proposals.

## Zenoh warnings about a missing router

`rmw_zenoh_cpp` needs a router. Run:

```bash
pixi run -e jazzy zenohd
```

or use `graspnet_zenoh.launch.py`. If the parent pipeline uses Fast DDS, set
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp`.

## Colcon tries to build the submodule

The wrapper already includes `third_party/COLCON_IGNORE` and the build task uses
`--base-paths src`. If you run `colcon build` manually, include
`--base-paths src`.

## Pixi install runs out of disk

Point the cache at a larger disk:

```bash
export PIXI_CACHE_DIR=/path/to/large/disk/pixi-cache
pixi install -e jazzy
```

## Docker build is slow

The image contains ROS 2, PyTorch CUDA and RViz. Use the GitHub Actions Docker
workflow's build cache, or keep a local BuildKit cache. The first build is
expected to be large.

## Checkpoint loads with a legacy pickle warning

The bundled checkpoint contains NumPy metadata. The engine retries with the
legacy pickle loader for this trusted local file. This is expected.
