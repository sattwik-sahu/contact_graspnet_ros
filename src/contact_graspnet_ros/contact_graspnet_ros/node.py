"""rclpy node for Contact-GraspNet inference."""

from __future__ import annotations

import contextlib
import threading
from dataclasses import replace

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
    qos_profile_system_default,
)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from contact_graspnet_msgs.msg import GraspProposal as GraspProposalMsg
from contact_graspnet_msgs.msg import GraspProposalArray
from contact_graspnet_msgs.srv import GenerateGrasps, GenerateGraspsFromPointClouds
from contact_graspnet_ros.config import NodeConfig
from contact_graspnet_ros.conversions import (
    camera_info_to_intrinsics,
    image_to_depth,
    image_to_mask,
    lookup_transform_matrix,
    make_contact_cloud,
    pointcloud2_to_xyz,
    pose_from_matrix,
    transform_point,
    transform_pose_matrix,
)
from contact_graspnet_ros.core.engine import ContactGraspNetEngine
from contact_graspnet_ros.core.types import GraspProposal, GraspResult, InferenceOptions
from contact_graspnet_ros.markers import build_marker_array


class ContactGraspNetNode(Node):
    """Topic- and service-based Contact-GraspNet wrapper."""

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
        self._dropped_frames = 0
        self._last_inference_ms = 0.0
        self._last_num_grasps = 0
        self._last_error = ""

        input_qos = _qos_profile(self._config.input_qos)
        output_qos = _qos_profile(self._config.output_qos)

        self._grasp_pub = self.create_publisher(
            GraspProposalArray, "grasps", output_qos
        )
        self._contact_pub = self.create_publisher(
            PointCloud2, "contact_points", output_qos
        )
        self._marker_pub = self.create_publisher(MarkerArray, "markers", output_qos)
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "diagnostics", output_qos
        )

        self.create_service(
            GenerateGrasps, "generate_grasps", self._handle_generate_grasps
        )
        self.create_service(
            GenerateGraspsFromPointClouds,
            "generate_grasps_from_pointclouds",
            self._handle_generate_grasps_from_pointclouds,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._depth_sub = Subscriber(
            self, Image, self._config.depth_topic, qos_profile=input_qos
        )
        self._info_sub = Subscriber(
            self, CameraInfo, self._config.camera_info_topic, qos_profile=input_qos
        )
        self._mask_sub = None
        if self._config.mask_topic:
            self._mask_sub = Subscriber(
                self, Image, self._config.mask_topic, qos_profile=input_qos
            )
            self._sync = ApproximateTimeSynchronizer(
                [self._depth_sub, self._info_sub, self._mask_sub],
                queue_size=5,
                slop=self._config.sync_slop_s,
            )
            self._sync.registerCallback(self._on_synced_with_mask)
        else:
            self._sync = ApproximateTimeSynchronizer(
                [self._depth_sub, self._info_sub],
                queue_size=5,
                slop=self._config.sync_slop_s,
            )
            self._sync.registerCallback(self._on_synced)

        self._diag_timer = self.create_timer(
            1.0 / self._config.diagnostics_rate_hz, self._publish_diagnostics
        )

        self.get_logger().info(
            "Contact-GraspNet node ready: "
            f"depth={self._config.depth_topic}, camera_info={self._config.camera_info_topic}, "
            f"mask={self._config.mask_topic or '<disabled>'}"
        )

    # ------------------------------------------------------------------
    # Topic mode
    # ------------------------------------------------------------------
    def _on_synced(self, depth_msg: Image, info_msg: CameraInfo) -> None:
        self._handle_frame(depth_msg, info_msg, None)

    def _on_synced_with_mask(
        self, depth_msg: Image, info_msg: CameraInfo, mask_msg: Image
    ) -> None:
        self._handle_frame(depth_msg, info_msg, mask_msg)

    def _handle_frame(
        self, depth_msg: Image, info_msg: CameraInfo, mask_msg: Image | None
    ) -> None:
        if not self._inference_lock.acquire(blocking=False):
            self._dropped_frames += 1
            return
        try:
            mask = None
            if mask_msg is not None:
                mask = image_to_mask(
                    mask_msg, binary=self._config.mask_mode == "binary"
                )
            result = self._run_inference(
                depth_msg, info_msg, mask, self._config.inference_options()
            )
            frame_id = self._resolve_frame_id(depth_msg, info_msg)
            result, output_frame = self._transform_result(
                result, frame_id, depth_msg.header.stamp
            )
            self._publish_result(result, output_frame, depth_msg.header.stamp)
        except Exception as exc:  # noqa: BLE001 - a frame must not kill the node
            self._last_error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"Grasp inference failed: {self._last_error}")
        finally:
            self._inference_lock.release()

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------
    def _handle_generate_grasps(self, request, response):
        if not self._inference_lock.acquire(timeout=self._config.service_timeout_s):
            response.success = False
            response.message = "busy: inference is already running"
            return response

        try:
            mask = None
            if request.mask.width > 0 and request.mask.height > 0:
                mask = image_to_mask(
                    request.mask, binary=self._config.mask_mode == "binary"
                )
            options = self._request_options(request)
            result = self._run_inference(
                request.depth, request.camera_info, mask, options
            )
            frame_id = self._resolve_frame_id(request.depth, request.camera_info)
            response.grasps = self._to_message(
                result, frame_id, request.depth.header.stamp
            )
            response.success = True
            response.message = f"{len(result)} grasps"
            self._update_counters(result)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(f"GenerateGrasps failed: {self._last_error}")
            response.success = False
            response.message = self._last_error
        finally:
            self._inference_lock.release()
        return response

    def _handle_generate_grasps_from_pointclouds(self, request, response):
        if not self._inference_lock.acquire(timeout=self._config.service_timeout_s):
            response.success = False
            response.message = "busy: inference is already running"
            return response

        try:
            points = pointcloud2_to_xyz(request.cloud)
            segment = None
            if request.segment.width > 0 and request.segment.height > 0:
                segment = pointcloud2_to_xyz(request.segment)
            options = self._request_options(request)
            result = self._engine.infer_pointclouds(points, segment, options)
            frame_id = request.cloud.header.frame_id
            if not frame_id:
                raise ValueError("PointCloud2 header.frame_id is empty")
            response.grasps = self._to_message(
                result, frame_id, request.cloud.header.stamp
            )
            response.success = True
            response.message = f"{len(result)} grasps"
            self._update_counters(result)
        except Exception as exc:  # noqa: BLE001
            self._last_error = f"{type(exc).__name__}: {exc}"
            self.get_logger().error(
                f"GenerateGraspsFromPointClouds failed: {self._last_error}"
            )
            response.success = False
            response.message = self._last_error
        finally:
            self._inference_lock.release()
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _run_inference(
        self,
        depth_msg: Image,
        info_msg: CameraInfo,
        mask: np.ndarray | None,
        options: InferenceOptions,
    ) -> GraspResult:
        depth = image_to_depth(depth_msg, depth_scale=self._config.depth_scale)
        intrinsics = camera_info_to_intrinsics(info_msg)
        result = self._engine.infer(depth, intrinsics, mask, options)
        self._update_counters(result)
        if result.warnings:
            self.get_logger().warning("; ".join(result.warnings))
        return result

    def _request_options(self, request) -> InferenceOptions:
        base = self._config.inference_options()
        forward_passes = (
            int(request.forward_passes)
            if request.forward_passes
            else base.forward_passes
        )
        z_range = getattr(request, "z_range", base.z_range)
        skip_border_objects = getattr(
            request, "skip_border_objects", base.skip_border_objects
        )
        return replace(
            base,
            z_range=(float(z_range[0]), float(z_range[1])),
            local_regions=bool(request.local_regions),
            filter_grasps=bool(request.filter_grasps),
            skip_border_objects=bool(skip_border_objects),
            segmap_id=int(request.segmap_id),
            forward_passes=forward_passes,
        )

    def _resolve_frame_id(self, depth_msg: Image, info_msg: CameraInfo) -> str:
        frame_id = info_msg.header.frame_id or depth_msg.header.frame_id
        if not frame_id:
            raise ValueError("depth/camera_info header.frame_id is empty")
        return frame_id

    def _transform_result(
        self, result: GraspResult, source_frame: str, stamp
    ) -> tuple[GraspResult, str]:
        target_frame = self._config.output_frame
        if not target_frame or target_frame == source_frame:
            return result, source_frame

        transform = lookup_transform_matrix(
            self._tf_buffer,
            target_frame,
            source_frame,
            Time.from_msg(stamp),
            self._config.tf_timeout_s,
        )
        proposals: list[GraspProposal] = []
        for proposal in result.proposals:
            proposals.append(
                replace(
                    proposal,
                    pose=transform_pose_matrix(proposal.pose, transform),
                    contact_point=transform_point(proposal.contact_point, transform),
                )
            )
        return (
            GraspResult(
                proposals=tuple(proposals),
                num_input_points=result.num_input_points,
                inference_time_s=result.inference_time_s,
                warnings=result.warnings,
            ),
            target_frame,
        )

    def _publish_result(self, result: GraspResult, frame_id: str, stamp) -> None:
        self._grasp_pub.publish(self._to_message(result, frame_id, stamp))
        header = Header(stamp=stamp, frame_id=frame_id)
        if self._config.publish_contact_points and result.proposals:
            contacts = np.asarray(
                [proposal.contact_point for proposal in result.proposals],
                dtype=np.float32,
            )
            self._contact_pub.publish(make_contact_cloud(header, contacts))
        if self._config.publish_markers:
            self._marker_pub.publish(build_marker_array(result, frame_id, stamp))

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

    def _update_counters(self, result: GraspResult) -> None:
        self._last_inference_ms = float(result.inference_time_s * 1000.0)
        self._last_num_grasps = len(result)
        if not result.proposals:
            self._last_error = ""

    def _publish_diagnostics(self) -> None:
        status = DiagnosticStatus()
        status.name = "contact_graspnet"
        status.hardware_id = self._engine.device_name
        if self._last_error:
            status.level = DiagnosticStatus.WARN
            status.message = self._last_error
        else:
            status.level = DiagnosticStatus.OK
            status.message = "running"
        status.values = [
            KeyValue(key="device", value=self._engine.device_name),
            KeyValue(key="checkpoint", value=str(self._config.checkpoint_path)),
            KeyValue(key="inference_ms", value=f"{self._last_inference_ms:.2f}"),
            KeyValue(key="num_grasps", value=str(self._last_num_grasps)),
            KeyValue(key="dropped_frames", value=str(self._dropped_frames)),
            KeyValue(key="last_error", value=self._last_error),
        ]
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._config.output_frame
        message.status = [status]
        self._diag_pub.publish(message)

    def destroy_node(self) -> bool:
        with contextlib.suppress(Exception):
            self._engine.close()
        return super().destroy_node()


def _qos_profile(name: str) -> QoSProfile:
    if name == "sensor_data":
        return qos_profile_sensor_data
    if name == "default":
        return qos_profile_system_default
    if name == "reliable":
        return QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
    raise ValueError(f"Unsupported QoS profile {name!r}")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: ContactGraspNetNode | None = None
    try:
        node = ContactGraspNetNode()
        executor = MultiThreadedExecutor(num_threads=3)
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
