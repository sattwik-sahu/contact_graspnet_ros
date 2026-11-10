---
title: Inference Contract
description: Inputs, outputs, shapes, dtypes and pose conventions.
permalink: /inference-contract/
layout: default
parent: Reference
nav_order: 3
---
## Inputs

| Name | Shape | dtype | Units/frame | Notes |
|---|---|---|---|---|
| `depth` | `(H, W)` | `float32` | metres, camera optical | `32FC1` or `16UC1` scaled by `depth_scale` |
| `K` | `(3, 3)` | `float32` | pixels | `CameraInfo.P` preferred, `K` fallback |
| `mask` | `(H, W)` | `int32` | labels, 0 = background | optional |

## Model outputs

| Name | Shape | dtype | Notes |
|---|---|---|---|
| `pred_grasps_cam` | `(B, N, 4, 4)` | `float32` | gripper-base poses |
| `pred_scores` | `(B, N, 1)` | `float32` | contact confidence |
| `pred_points` | `(B, N, 3)` | `float32` | contact candidates |
| `offset_pred` | `(B, N, 1)` | `float32` | predicted finger opening |

## Engine output

```python
GraspResult(
    proposals=(
        GraspProposal(
            pose=np.ndarray,          # (4, 4) float32
            score=float,              # [0, 1]
            width=float,              # metres
            contact_point=np.ndarray, # (3,) float32
            segment_id=int,           # -1 for full scene
        ),
        ...
    ),
    num_input_points=int,
    inference_time_s=float,
    warnings=tuple[str, ...],
)
```

Proposals are sorted by descending score.

## Pose convention

The pose is the gripper base frame in the camera optical frame:

- `+Z`: approach direction
- `+X`: finger-closing direction
- `+Y`: `Z x X`
- translation: `contact + width/2 * closing - 0.1034 m * approach`

`width` is the commanded finger opening, clamped to the model's gripper width.

## Coordinate conversion

The network is trained in an internal frame with x left, y up, z forward. The
engine flips x/y before the forward pass and flips them back afterwards, so the
public API is in the OpenCV/REP 103 optical frame.
