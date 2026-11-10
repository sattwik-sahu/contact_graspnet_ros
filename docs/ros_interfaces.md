# ROS interfaces

## Topics

| Direction | Default name | Type | QoS |
|---|---|---|---|
| Input | `depth` | `sensor_msgs/Image` | sensor data |
| Input | `camera_info` | `sensor_msgs/CameraInfo` | sensor data |
| Input | `mask` | `sensor_msgs/Image` | sensor data |
| Output | `grasps` | `contact_graspnet_msgs/GraspProposalArray` | reliable |
| Output | `contact_points` | `sensor_msgs/PointCloud2` | reliable |
| Output | `markers` | `visualization_msgs/MarkerArray` | reliable |
| Output | `diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable |

The launch file uses the namespace `graspnet` by default, so the full names are
`/graspnet/depth`, `/graspnet/camera_info`, `/graspnet/mask`, `/graspnet/grasps`,
`/graspnet/contact_points`, `/graspnet/markers` and `/graspnet/diagnostics`.
Services are `/graspnet/generate_grasps` and
`/graspnet/generate_grasps_from_pointclouds`.

## Messages

### `GraspProposal.msg`

```text
geometry_msgs/Pose pose
float32 score
float32 width
geometry_msgs/Point contact_point
int32 segment_id
```

### `GraspProposalArray.msg`

```text
std_msgs/Header header
uint32 num_input_points
float32 inference_time_s
GraspProposal[] grasps
```

## Services

### `GenerateGrasps.srv`

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

### `GenerateGraspsFromPointClouds.srv`

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

For services, use `segmap_id > 0` when supplying a segment cloud. An empty
mask/segment means no segmentation, in which case `local_regions` and
`filter_grasps` must be false.
