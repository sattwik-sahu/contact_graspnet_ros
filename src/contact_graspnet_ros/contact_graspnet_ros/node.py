"""Service-only rclpy node for Contact-GraspNet inference."""

from __future__ import annotations

import contextlib
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from contact_graspnet_msgs.msg import GraspProposal as GraspProposalMsg
from contact_graspnet_msgs.msg import GraspProposalArray
from contact_graspnet_msgs.srv import GenerateGrasps
from contact_graspnet_ros.config import NodeConfig
from contact_graspnet_ros.conversions import (
    camera_info_to_intrinsics,
    image_to_depth,
    image_to_mask,
    pose_from_matrix,
)
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.types import GraspResult, InferenceOptions


class ContactGraspNetNode(Node):
    """Expose Contact-GraspNet as a single request/response ROS 2 service."""

    def __init__(self) -> None:
        super().__init__("contact_graspnet")

        self._config = NodeConfig.declare_and_read(self)
        self.get_logger().info(
            f"Loading Contact-GraspNet from {self._config.checkpoint_path} "
            f"(device={self._config.device})"
        )
        self._engine = ContactGraspNetEngine(
            model_config_path=str(self._config.model_config_path),
            checkpoint_path=str(self._config.checkpoint_path),
            device=self._config.device,
            seed=self._config.seed,
        )
        self.get_logger().info(
            f"Engine ready on {self._engine.device_name}; "
            f"raw_num_points={self._engine.defaults.raw_num_points}"
        )

        self._inference_lock = threading.Lock()
        self.create_service(
            GenerateGrasps,
            "generate_grasps",
            self._handle_generate_grasps,
        )
        self.get_logger().info(
            f"Service ready: {self.get_namespace().rstrip('/')}/generate_grasps"
        )

    def _handle_generate_grasps(
        self, request: GenerateGrasps.Request, response: GenerateGrasps.Response
    ) -> GenerateGrasps.Response:
        """Run inference for one depth/mask request.

        Requests are serialized: a second call waits up to
        ``service_timeout_s`` and then returns a busy failure.
        """

        if not self._inference_lock.acquire(timeout=self._config.service_timeout_s):
            response.success = False
            response.message = "busy: inference is already running"
            return response

        try:
            depth = image_to_depth(request.depth, depth_scale=self._config.depth_scale)
            intrinsics = camera_info_to_intrinsics(request.camera_info)

            mask = None
            if request.mask.width > 0 and request.mask.height > 0:
                mask = image_to_mask(
                    request.mask, binary=self._config.mask_mode == "binary"
                )

            frame_id = self._resolve_frame_id(request.depth, request.camera_info)
            options = self._request_options(request)
            result = self._engine.infer(depth, intrinsics, mask, options)

            response.grasps = self._to_message(
                result, frame_id, request.depth.header.stamp
            )
            response.success = True
            response.message = f"{len(result)} grasps"
            self.get_logger().info(
                f"{len(result)} grasps in {result.inference_time_s:.3f}s "
                f"(frame={frame_id}, points={result.num_input_points})"
            )
            for warning in result.warnings:
                self.get_logger().warning(warning)
        except Exception as exc:  # noqa: BLE001 - service errors must not kill the node
            response.success = False
            response.message = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"GenerateGrasps failed: {response.message}")
        finally:
            self._inference_lock.release()

        return response

    def _request_options(self, request: GenerateGrasps.Request) -> InferenceOptions:
        base = self._config.inference_options()
        forward_passes = (
            int(request.forward_passes)
            if request.forward_passes
            else base.forward_passes
        )
        return InferenceOptions(
            z_range=(float(request.z_range[0]), float(request.z_range[1])),
            local_regions=bool(request.local_regions),
            filter_grasps=bool(request.filter_grasps),
            skip_border_objects=bool(request.skip_border_objects),
            segmap_id=int(request.segmap_id),
            max_segments=base.max_segments,
            forward_passes=forward_passes,
            first_thres=base.first_thres,
            second_thres=base.second_thres,
            num_samples=base.num_samples,
            max_farthest_points=base.max_farthest_points,
        )

    def _resolve_frame_id(self, depth_msg: Image, info_msg: CameraInfo) -> str:
        frame_id = depth_msg.header.frame_id or info_msg.header.frame_id
        if not frame_id:
            raise ValueError("depth/camera_info header.frame_id is empty")
        return frame_id

    def _to_message(
        self, result: GraspResult, frame_id: str, stamp
    ) -> GraspProposalArray:
        message = GraspProposalArray()
        message.header = Header(stamp=stamp, frame_id=frame_id)
        message.num_input_points = int(result.num_input_points)
        message.inference_time_s = float(result.inference_time_s)
        for proposal in result.proposals:
            message.grasps.append(
                GraspProposalMsg(
                    pose=pose_from_matrix(proposal.pose),
                    score=float(proposal.score),
                    width=float(proposal.width),
                    contact_point=Point(
                        x=float(proposal.contact_point[0]),
                        y=float(proposal.contact_point[1]),
                        z=float(proposal.contact_point[2]),
                    ),
                    segment_id=int(proposal.segment_id),
                )
            )
        return message

    def destroy_node(self) -> bool:
        with contextlib.suppress(Exception):
            self._engine.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ContactGraspNetNode | None = None
    try:
        node = ContactGraspNetNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
