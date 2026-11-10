# Configuration

All pipeline configuration lives in the repository's `config/` directory.

```text
config/
├── graspnet.yaml     # ROS service parameters
└── model.yaml        # model architecture and default thresholds
```

## Model config

`config/model.yaml` is a copy of the vendored port's checkpoint configuration.
It is authoritative for model architecture. Do not edit architecture values
unless you are using a matching checkpoint.

The port's default selection thresholds are:

```yaml
TEST:
  first_thres: 0.15
  second_thres: 0.15
```

These can be overridden by ROS parameters at runtime.

## ROS parameters

Key parameters in `config/graspnet.yaml`:

| Parameter | Default | Notes |
|---|---|---|
| `model_config` | `model.yaml` | relative to `GRASPNET_CONFIG_DIR` |
| `checkpoint_dir` | vendored checkpoint dir | directory containing `model.pt` |
| `device` | `cuda` | `cuda`, `cpu` or `auto` |
| `z_range` | `[0.2, 1.8]` | metres |
| `local_regions` | `true` | requires a request mask |
| `filter_grasps` | `true` | requires a request mask |
| `skip_border_objects` | `false` | |
| `segmap_id` | `0` | `0` = all positive labels |
| `max_segments` | `4` | safety cap for crowded masks |
| `forward_passes` | `1` | number of batched model passes |
| `first_thres` | `0.15` | contact confidence threshold |
| `second_thres` | `0.15` | fill-up confidence threshold |
| `num_samples` | `200` | max grasps per segment |
| `max_farthest_points` | `150` | farthest-point selection count |
| `seed` | `0` | RNG seed |
| `service_timeout_s` | `5.0` | concurrent-call wait before `busy` |
| `mask_mode` | `labels` | `labels` or `binary` |
| `depth_scale` | `0.001` | metres per unit for `16UC1` depth |

The `GenerateGrasps` request can override `z_range`, `local_regions`,
`filter_grasps`, `skip_border_objects`, `segmap_id` and `forward_passes` for a
single call.

Startup-only parameters such as `checkpoint_dir`, `model_config` and `device`
should be changed in the YAML and the node restarted.

## Calling without a mask

Set the request fields:

```python
request.mask = Image()          # empty
request.local_regions = False
request.filter_grasps = False
```

The service then runs full-scene inference.

## Path overrides

| Variable | Purpose |
|---|---|
| `GRASPNET_CONFIG_DIR` | directory containing `graspnet.yaml` and `model.yaml` |
| `GRASPNET_CHECKPOINT_DIR` | directory containing `model.pt` |
| `CONTACT_GRASPNET_ROOT` | repository root used to locate the vendored port |

Pixi sets these in `scripts/activate.sh`. Docker sets them in the Compose
service. The Docker config mount is read-only, so a parameter change only needs
a container restart, not a rebuild.
