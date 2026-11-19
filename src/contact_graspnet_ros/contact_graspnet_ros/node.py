"""Service-only rclpy node for Contact-GraspNet inference."""

from __future__ import annotations

import contextlib
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from contact_graspnet_msgs.msg import GraspProposal as GraspProposalMsg
from contact_graspnet_msgs.msg import GraspProposalArray
from contact_graspnet_msgs.srv import GenerateGrasps
from contact_graspnet_ros.config import NodeConfig
from contact_graspnet_ros.conversions import (
    camera_info_to_intrinsics,
    image_to_depth,
    image_to_mask,
    matrix_from_transform_stamped,
    pose_from_matrix,
)
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.transforms import (
    apply_transform,
    transform_from_offset_rpy,
)
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

        self._tf_buffer = Buffer()
        # Give the listener its own node and thread: a service callback blocks
        # while waiting for a transform, and sharing this node's executor would
        # either deadlock or double-spin the node's subscriptions.
        self._tf_listener = TransformListener(
            self._tf_buffer, None, spin_thread=True
        )

        # Static transform from the network's gripper frame to the downstream
        # tool/EE frame. Identity by default (native gripper-base output).
        self._grasp_to_tool = transform_from_offset_rpy(
            self._config.tcp_offset, self._config.tcp_rpy
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

            camera_frame = self._resolve_frame_id(
                request.depth, request.camera_info
            )
            output_frame, target_from_camera = self._resolve_output_frame(
                request, camera_frame
            )
            options = self._request_options(request)
            result = self._engine.infer(depth, intrinsics, mask, options)

            response.grasps = self._to_message(
                result,
                output_frame,
                request.depth.header.stamp,
                target_from_camera,
            )
            response.success = True
            response.message = f"{len(result)} grasps"
            self.get_logger().info(
                f"{len(result)} grasps in {result.inference_time_s:.3f}s "
                f"(camera_frame={camera_frame}, output_frame={output_frame}, "
                f"points={result.num_input_points})"
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

    def _resolve_output_frame(
        self, request: GenerateGrasps.Request, camera_frame: str
    ) -> tuple[str, np.ndarray | None]:
        """Return the requested output frame and ``T_output_camera``.

        The request field wins, then the node parameter; empty means "keep the
        camera frame". ``None`` signals an identity transform.
        """

        output_frame = (
            request.target_frame.strip()
            or self._config.target_frame.strip()
            or camera_frame
        )
        if output_frame == camera_frame:
            return camera_frame, None

        stamp = request.depth.header.stamp
        lookup_time = (
            Time() if stamp.sec == 0 and stamp.nanosec == 0 else Time.from_msg(stamp)
        )
        try:
            transform = self._tf_buffer.lookup_transform(
                output_frame,
                camera_frame,
                lookup_time,
                timeout=Duration(seconds=self._config.tf_timeout_s),
            )
        except TransformException as exc:
            raise RuntimeError(
                f"cannot transform grasps from {camera_frame!r} to "
                f"{output_frame!r}: {exc}"
            ) from exc

        return output_frame, matrix_from_transform_stamped(transform)

    def _to_message(
        self,
        result: GraspResult,
        frame_id: str,
        stamp,
        target_from_camera: np.ndarray | None,
    ) -> GraspProposalArray:
        message = GraspProposalArray()
        message.header = Header(stamp=stamp, frame_id=frame_id)
        message.num_input_points = int(result.num_input_points)
        message.inference_time_s = float(result.inference_time_s)
        for proposal in result.proposals:
            pose = proposal.pose @ self._grasp_to_tool
            contact_point = proposal.contact_point
            if target_from_camera is not None:
                pose = target_from_camera @ pose
                contact_point = apply_transform(target_from_camera, contact_point)

            message.grasps.append(
                GraspProposalMsg(
                    pose=pose_from_matrix(pose),
                    score=float(proposal.score),
                    width=float(proposal.width),
                    contact_point=Point(
                        x=float(contact_point[0]),
                        y=float(contact_point[1]),
                        z=float(contact_point[2]),
                    ),
                    segment_id=int(proposal.segment_id),
                )
            )
        return message

    def destroy_node(self) -> bool:
        with contextlib.suppress(Exception):
            self._engine.close()
        with contextlib.suppress(Exception):
            self._tf_listener.executor.shutdown()
        with contextlib.suppress(Exception):
            self._tf_listener.node.destroy_node()
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
