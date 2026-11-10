"""Launch the service-only Contact-GraspNet node."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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

    return [
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
            OpaqueFunction(function=_launch_setup),
        ]
    )
