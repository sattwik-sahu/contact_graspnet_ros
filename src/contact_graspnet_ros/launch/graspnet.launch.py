"""Launch the Contact-GraspNet ROS 2 node."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_config_dir() -> str:
    return os.environ.get("GRASPNET_CONFIG_DIR", os.path.join(os.getcwd(), "config"))


def _launch_setup(context, *args, **kwargs):
    config_dir = LaunchConfiguration("config_dir").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)
    if not params_file:
        params_file = os.path.join(config_dir, "graspnet.yaml")

    checkpoint_dir = LaunchConfiguration("checkpoint_dir").perform(context)
    output_frame = LaunchConfiguration("output_frame").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)

    parameters = [
        params_file,
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            )
        },
    ]
    if checkpoint_dir:
        parameters.append({"checkpoint_dir": checkpoint_dir})
    if output_frame:
        parameters.append({"output_frame": output_frame})

    actions = [
        Node(
            package="contact_graspnet_ros",
            executable="graspnet_node",
            name="contact_graspnet",
            namespace=namespace,
            parameters=parameters,
            arguments=["--ros-args", "--log-level", log_level],
            output="screen",
            emulate_tty=True,
        )
    ]

    if LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes"):
        rviz_config = os.path.join(config_dir, "rviz", "graspnet.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="graspnet_rviz",
                namespace=namespace,
                arguments=["-d", rviz_config],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_dir",
                default_value=_default_config_dir(),
                description="Directory containing graspnet.yaml and model.yaml",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="",
                description="Explicit ROS parameter file; empty means config_dir/graspnet.yaml",
            ),
            DeclareLaunchArgument(
                "checkpoint_dir",
                default_value=os.environ.get("GRASPNET_CHECKPOINT_DIR", ""),
                description="Directory containing model.pt; empty uses the YAML value",
            ),
            DeclareLaunchArgument(
                "output_frame",
                default_value="",
                description="Optional TF target frame; empty keeps the camera optical frame",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="graspnet",
                description="Node namespace",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use simulation time"
            ),
            DeclareLaunchArgument(
                "log_level", default_value="info", description="ROS log level"
            ),
            DeclareLaunchArgument(
                "rviz", default_value="false", description="Start RViz2"
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
