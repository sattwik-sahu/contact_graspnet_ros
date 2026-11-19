# Grasp Frames and Downstream Planning

This package is a **perception** component: it turns depth + intrinsics +
optional mask into 6-DoF grasp proposals. It does not plan or execute motion.
This page describes the output contract so that a downstream planner (MoveIt,
a custom IK node, or anything else) can consume the proposals directly.

Two independent settings control the output:

| Setting | What it changes | Where |
|---|---|---|
| `target_frame` | The **TF frame** the poses are expressed in | request field and node parameter |
| `tcp_offset`, `tcp_rpy` | The **static transform** from the network's gripper frame to your tool/EE frame | node parameters |

They are applied in this order:

```text
T_output_tool = T_target_camera @ T_camera_grasp @ T_grasp_tool
```

where `T_camera_grasp` is the raw network pose and `T_grasp_tool` is built from
`tcp_offset`/`tcp_rpy`.

---

## 1. Choosing the output frame

Set `GenerateGrasps.target_frame` to the frame you want to plan in, for example
`base_link`, `world`, or `odom`:

```python
request.target_frame = "base_link"
```

Resolution order:

1. `request.target_frame`, if non-empty;
2. the node parameter `target_frame`, if non-empty;
3. otherwise the depth/camera frame (legacy behaviour).

The response's `grasps.header.frame_id` is set to the resolved frame, and every
`pose` and `contact_point` is expressed in it.

### Requirements

- TF must contain a transform from the **depth/camera frame** to the target
  frame. The depth frame is `request.depth.header.frame_id`, falling back to
  `camera_info.header.frame_id`.
- The lookup uses `request.depth.header.stamp`. A zero stamp means "latest
  available transform".
- The lookup waits up to `tf_timeout_s` (default `0.2` s). If it fails, the
  service returns `success: false` with a message naming both frames.

For a fixed arm this is usually the full chain:

```text
base_link -> ... -> camera_link -> camera_optical
```

Make sure the frame in `depth.header.frame_id` is the optical frame published
by your camera driver.

### REP 103

The target frame is commonly a REP 103 frame (X forward, Y left, Z up). You do
**not** need to permute axes by hand: expressing the pose in that frame is one
rigid TF transform, and the rotation matrix that comes out is already the
gripper's rotation relative to that frame. Manually reordering rows/columns
would corrupt the pose.

---

## 2. The tool frame (`tcp_offset` and `tcp_rpy`)

The network returns a **gripper-base** pose with its own axis convention:

- `+X` — finger-closing direction
- `+Z` — approach direction (gripper toward the object)
- `+Y` — `Z × X`
- origin at the mounting flange; the finger contact line is `0.1034 m` along
  `+Z`

Your downstream planner moves a specific **tip/EE link**, and that link may have
a different origin and axis convention. `tcp_offset` and `tcp_rpy` describe that
difference:

```yaml
tcp_offset: [x, y, z]   # metres, expressed in the network gripper frame
tcp_rpy:    [r, p, y]   # radians, fixed-axis roll/pitch/yaw
```

The transform is built as `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`.

### Common choices

| Downstream EE link | `tcp_offset` | `tcp_rpy` | Notes |
|---|---|---|---|
| Native Contact-GraspNet frame | `[0, 0, 0]` | `[0, 0, 0]` | Keeps `+X` closing / `+Z` approach |
| Franka `panda_hand` | `[0, 0, 0]` | `[0, 0, 1.5708]` | Fingers translate along `+Y` of `panda_hand` |
| Franka `panda_link8` | `[0, 0, 0]` | `[0, 0, 2.3562]` | `panda_hand` is mounted with `rpy="0 0 -0.785398"` on `panda_link8` |
| Franka fingertip / contact line | `[0, 0, 0.1034]` | `[0, 0, 1.5708]` | Origin moved to the finger contact line |
| Custom / unknown gripper | calibrate | calibrate | See the procedure below |

The Panda values come from the stock `franka_description` URDF:

- `panda_hand` is a fixed child of `panda_link8` with
  `origin rpy="0 0 -0.785398" xyz="0 0 0"`;
- `panda_finger_joint1/2` are prismatic joints with `axis xyz="0 1 0"`.

So GraspNet's `+X` closing axis must map to `+Y` of `panda_hand`: a `+90°` yaw.
`panda_hand` is rotated `-45°` relative to `panda_link8`, so planning for
`panda_link8` adds another `+45°`, giving `135°` (`2.3562` rad).

> The sign is a convention, not a law. Always verify with the procedure below;
> different URDFs or a rotated hand mount can flip it.

### Calibrating a custom gripper

1. Start the node with `target_frame: base_link` and identity
   `tcp_offset`/`tcp_rpy`.
2. Call the service for a single object.
3. Load your gripper URDF/mesh in RViz at each returned pose.
4. Adjust `tcp_rpy` until the fingers straddle the contact pair
   — the two finger contact points are `contact_point` and
   `contact_point + width * (pose +X axis)`.
5. Adjust `tcp_offset` until the fingertip plane sits on that contact line.
   For a parallel-jaw gripper this is usually a positive `z` of about
   `0.09`–`0.11 m`.
6. Restart the node and re-check.

You can also derive it from the URDF as
`T_grasp_tool = T_grasp_hand @ T_hand_tool` and convert the result to
`offset`/`rpy`, but visual verification is still recommended.

---

## 3. End-to-end pipeline

```text
camera driver
  ├─ sensor_msgs/Image depth          (32FC1 metres or 16UC1 mm)
  ├─ sensor_msgs/CameraInfo           (P and/or K)
  └─ TF: base_link -> ... -> camera_optical
        │
segmentation (2D or 3D)
  └─ sensor_msgs/Image mask           (mono8/mono16/32SC1, 0 = background)
        │
        ▼
/graspnet/generate_grasps  (this package)
        │  target_frame: "base_link"
        │  tcp_offset / tcp_rpy
        ▼
GraspProposalArray
  header.frame_id = "base_link"
  grasps[i].pose, score, width, contact_point
        │
        ▼
downstream planner (MoveIt / custom IK / etc.)
```

### 3.1 Start the node with frame settings

Either edit `config/graspnet.yaml`:

```yaml
/**:
  ros__parameters:
    target_frame: "base_link"
    tcp_offset: [0.0, 0.0, 0.0]
    tcp_rpy: [0.0, 0.0, 1.5708]
    tf_timeout_s: 0.2
```

or pass them at launch:

```bash
ros2 run contact_graspnet_ros graspnet_node --ros-args \
  -p target_frame:=base_link \
  -p tcp_offset:="[0.0, 0.0, 0.0]" \
  -p tcp_rpy:="[0.0, 0.0, 1.5708]"
```

or override per request (see below). The request always wins.

### 3.2 Call the service

```python
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo
from contact_graspnet_msgs.srv import GenerateGrasps

bridge = CvBridge()

req = GenerateGrasps.Request()
req.depth = bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
req.depth.header.frame_id = "camera_optical"
req.depth.header.stamp = node.get_clock().now().to_msg()

req.camera_info = CameraInfo()
req.camera_info.width = depth_m.shape[1]
req.camera_info.height = depth_m.shape[0]
req.camera_info.k = K.reshape(-1).tolist()

req.mask = bridge.cv2_to_imgmsg(segmap.astype(np.uint8), encoding="mono8")
req.segmap_id = 1
req.local_regions = True
req.filter_grasps = True
req.skip_border_objects = False
req.z_range = [0.2, 1.5]
req.forward_passes = 1
req.target_frame = "base_link"        # <-- output frame

response = client.call(req)
assert response.success, response.message
```

### 3.3 Consume the result downstream

```python
import copy

from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

for grasp in response.grasps.grasps:
    target = PoseStamped()
    target.header = response.grasps.header     # already base_link + stamp
    target.pose = grasp.pose                   # ready for MoveIt
    # grasp.width -> gripper command (subtract a small safety margin)
    # grasp.score -> ranking / filtering
    # grasp.contact_point -> visualisation or approach checks

    # Pregrasp: back off along the grasp's own +Z (approach) axis.
    # Deep-copy the pose: ROS 2 message assignment shares the nested object.
    q = grasp.pose.orientation
    R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    pregrasp = PoseStamped()
    pregrasp.header = target.header
    pregrasp.pose = copy.deepcopy(target.pose)
    pregrasp.pose.position.x -= 0.10 * R[0, 2]
    pregrasp.pose.position.y -= 0.10 * R[1, 2]
    pregrasp.pose.position.z -= 0.10 * R[2, 2]
```

---

## 4. Verifying the result

The bundled client prints the frame, the top pose, and its contact point:

```bash
pixi run -e jazzy python scripts/call_grasp_service.py \
  --input third_party/contact_graspnet_pytorch/test_data/7.npy \
  --segmap-id 1 \
  --target-frame base_link
```

Expected output includes:

```text
frame: base_link
top_grasp: score=... width=... position=(...) orientation=(...) contact=(...)
```

Check that `tf2_echo` reports the same transform for the camera frame:

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical
```

---

## 5. Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `cannot transform grasps from 'camera_optical' to 'base_link'` | No TF, wrong frame name, or stale timestamp | Check `ros2 run tf2_ros tf2_echo base_link camera_optical`; verify `depth.header.frame_id` |
| `ExtrapolationException` | Depth stamp is ahead/behind the TF buffer | Use a zero stamp, increase `tf_timeout_s`, or check `use_sim_time` |
| Gripper is rotated 90° about the approach axis | `tcp_rpy` yaw is missing or has the wrong sign | Try `[0, 0, ±1.5708]` |
| Fingertips sit ~10 cm behind the object | `tcp_offset` is missing the contact-line offset | Add `0.1034` to the `z` component |
| Pose points into the object / away from it | Approach sign convention of your tool | Flip roll/pitch by `pi`, then re-verify |
| `busy: inference is already running` | A second request arrived | Serialize calls or raise `service_timeout_s` |

---

## 6. What this package does not do

- It does not run IK, collision checking, or trajectory planning.
- It does not know your robot model beyond the `tcp_offset`/`tcp_rpy` you give
  it.
- It does not choose the planning frame; pass `target_frame` and keep the two
  in sync.
