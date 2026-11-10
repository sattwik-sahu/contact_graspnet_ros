#!/usr/bin/env python3
"""Call the Contact-GraspNet ROS 2 service with an .npz/.npy scene.

Example:
    pixi run -e jazzy python scripts/call_grasp_service.py \
        --input third_party/contact_graspnet_pytorch/test_data/7.npy \
        --segmap-id 1
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

from contact_graspnet_msgs.srv import GenerateGrasps


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help=".npz/.npy scene")
    parser.add_argument("--service", default="/graspnet/generate_grasps")
    parser.add_argument("--segmap-id", type=int, default=1)
    parser.add_argument("--z-min", type=float, default=0.2)
    parser.add_argument("--z-max", type=float, default=2.0)
    parser.add_argument("--local-regions", action="store_true", default=True)
    parser.add_argument("--filter-grasps", action="store_true", default=True)
    parser.add_argument("--skip-border-objects", action="store_true", default=False)
    parser.add_argument("--forward-passes", type=int, default=1)
    parser.add_argument("--timeout", type=float, default=60.0)
    return parser.parse_args()


def _load_scene(path: Path):
    data = np.load(path, allow_pickle=True)
    if hasattr(data, "files"):
        keys = data.files
        depth = np.asarray(data["depth"], dtype=np.float32) if "depth" in keys else None
        K = (
            np.asarray(data["K"], dtype=np.float64).reshape(3, 3)
            if "K" in keys
            else None
        )
        mask_key = "segmap" if "segmap" in keys else "seg" if "seg" in keys else None
        mask = np.asarray(data[mask_key]) if mask_key else None
        return depth, K, mask

    array = np.asarray(data)
    if array.ndim == 0:
        item = array.item()
        if not isinstance(item, dict):
            raise ValueError(f"Unsupported 0-d input of type {type(item)}")
        depth = np.asarray(item["depth"], dtype=np.float32) if "depth" in item else None
        K = (
            np.asarray(item["K"], dtype=np.float64).reshape(3, 3)
            if "K" in item
            else None
        )
        mask_key = "segmap" if "segmap" in item else "seg" if "seg" in item else None
        mask = np.asarray(item[mask_key]) if mask_key else None
        return depth, K, mask

    raise ValueError(f"Expected a scene dictionary, got shape {array.shape}")


def main() -> None:
    args = _parse_args()
    depth, K, mask = _load_scene(args.input)
    if depth is None or K is None:
        raise SystemExit("Input must contain 'depth' and 'K'")
    if mask is not None:
        mask = mask.astype(np.uint8)

    bridge = CvBridge()
    rclpy.init()
    node = Node("graspnet_service_client")
    client = node.create_client(GenerateGrasps, args.service)
    if not client.wait_for_service(timeout_sec=15.0):
        raise SystemExit(f"Service not available: {args.service}")

    request = GenerateGrasps.Request()
    request.depth = bridge.cv2_to_imgmsg(depth, encoding="32FC1")
    request.depth.header.frame_id = "camera_optical"
    request.camera_info = CameraInfo()
    request.camera_info.width = int(depth.shape[1])
    request.camera_info.height = int(depth.shape[0])
    request.camera_info.k = K.reshape(-1).tolist()
    request.camera_info.p = [
        K[0, 0],
        0.0,
        K[0, 2],
        0.0,
        0.0,
        K[1, 1],
        K[1, 2],
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
    ]

    if mask is not None:
        request.mask = bridge.cv2_to_imgmsg(mask, encoding="mono8")
        request.segmap_id = args.segmap_id
        request.local_regions = args.local_regions
        request.filter_grasps = args.filter_grasps
    else:
        request.segmap_id = 0
        request.local_regions = False
        request.filter_grasps = False

    request.skip_border_objects = args.skip_border_objects
    request.z_range = [args.z_min, args.z_max]
    request.forward_passes = args.forward_passes

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
    response = future.result()
    if response is None:
        raise SystemExit("Service call timed out")

    print(f"success: {response.success}")
    print(f"message: {response.message}")
    print(f"frame: {response.grasps.header.frame_id}")
    print(f"num_grasps: {len(response.grasps.grasps)}")
    print(f"num_input_points: {response.grasps.num_input_points}")
    print(f"inference_time_s: {response.grasps.inference_time_s:.4f}")
    if response.grasps.grasps:
        top = max(response.grasps.grasps, key=lambda grasp: grasp.score)
        print(
            "top_grasp: "
            f"score={top.score:.4f} width={top.width:.4f} "
            f"segment={top.segment_id} "
            f"position=({top.pose.position.x:.4f}, "
            f"{top.pose.position.y:.4f}, {top.pose.position.z:.4f})"
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
