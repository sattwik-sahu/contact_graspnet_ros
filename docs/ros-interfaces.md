# ROS Interfaces

The wrapper is **service-only**. It does not subscribe to sensor topics or
stream output topics. A caller supplies a depth image, camera intrinsics and an
optional mask, and receives a list of grasp proposals.

## Service

```text
/graspnet/generate_grasps
  type: contact_graspnet_msgs/srv/GenerateGrasps
```

The node is launched in the `graspnet` namespace by default, so the service is
`/graspnet/generate_grasps`. Remap or change the launch namespace if needed.

### Request

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
string target_frame
```

- `depth`: `32FC1` in metres or `16UC1` in millimetres (`depth_scale`).
- `camera_info`: `P` is preferred, `K` is the fallback.
- `mask`: optional. `mono8`, `mono16` or `32SC1`; `0` is background.
- `segmap_id`: positive target label; `0` processes all positive labels.
- `local_regions` / `filter_grasps`: require a mask.
- `z_range`: crop in camera z, metres.
- `forward_passes`: model batch passes.
- `target_frame`: TF frame for the returned poses and contact points. Empty
  falls back to the node's `target_frame` parameter, then to the depth frame.

### Response

```text
bool success
string message
contact_graspnet_msgs/GraspProposalArray grasps
```

`GraspProposalArray`:

```text
std_msgs/Header header
uint32 num_input_points
float32 inference_time_s
GraspProposal[] grasps
```

`GraspProposal`:

```text
geometry_msgs/Pose pose
float32 score
float32 width
geometry_msgs/Point contact_point
int32 segment_id
```

The response header carries the resolved output frame and the input timestamp.
Poses and contact points are expressed in that frame, with the configured
`tcp_offset`/`tcp_rpy` tool transform applied. With the defaults
(`target_frame: ""`, identity tool transform) this is the native gripper-base
pose in the camera optical frame. See [Inference Contract](inference-contract.md)
and [Grasp Frames](grasp-frames.md) for the full contract.

## Calling the service

Use the provided helper:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1
```

Or call it from your own `rclpy` client. Requests are serialized on the node; a
second concurrent call waits up to `service_timeout_s` and then returns
`success: false` with `message: busy`.
