# Architecture

```text
depth + camera_info + optional mask
        │
        ▼
rclpy node (contact_graspnet_ros/node.py)
  message_filters.ApproximateTimeSynchronizer
        │
        ▼
ContactGraspNetEngine (contact_graspnet_ros/core/engine.py)  ← ROS-free
  ├─ geometry.py   depth2pc, z crop, segment/local-region crop, selection
  ├─ model.py      upstream ContactGraspnet subclass + checkpoint loading
  └─ types.py      typed dataclasses
        │
        ▼
GraspResult (poses, scores, widths, contacts, segment ids)
        │
        ▼
conversions.py / markers.py / node.py
  GraspProposalArray, PointCloud2, MarkerArray, DiagnosticArray
```

## Why the core is ROS-free

- Unit tests run without sourcing ROS.
- An HTTP bridge or CLI can reuse the engine.
- A future LibTorch/TensorRT backend can replace `core/engine.py` without
  changing the ROS interface.

## Why the submodule is untouched

The upstream PyTorch port is a read-only dependency. It contains useful model
code but also dead TensorFlow methods, a training-mode inference path and
pyrender imports. The wrapper uses only the `ContactGraspnet` model class and
its checkpoint. All orchestration and fixes live in our code.

## Threading

- `MultiThreadedExecutor` with three threads.
- One `threading.Lock` guards the engine.
- Topic frames are dropped when the lock is busy.
- Services wait up to `service_timeout_s`, then return a busy failure.
- This prevents unbounded queues and GPU oversubscription.

## Language choice

The ROS node is Python for now. The engine boundary is narrow so a C++/LibTorch
backend can be added later without changing topics, services or configuration.
