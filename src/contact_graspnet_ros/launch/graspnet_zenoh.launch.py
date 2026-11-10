"""Start the Zenoh router and the service-only Contact-GraspNet node."""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_launch = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "graspnet.launch.py"
    )

    router = Node(
        package="rmw_zenoh_cpp",
        executable="rmw_zenohd",
        name="rmw_zenohd",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_router")),
    )

    graspnet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(package_launch),
        launch_arguments={
            "config_dir": LaunchConfiguration("config_dir"),
            "checkpoint_dir": LaunchConfiguration("checkpoint_dir"),
            "namespace": LaunchConfiguration("namespace"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_dir",
                default_value=os.environ.get(
                    "GRASPNET_CONFIG_DIR", os.path.join(os.getcwd(), "config")
                ),
            ),
            DeclareLaunchArgument(
                "checkpoint_dir",
                default_value=os.environ.get("GRASPNET_CHECKPOINT_DIR", ""),
            ),
            DeclareLaunchArgument("namespace", default_value="graspnet"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("start_router", default_value="true"),
            router,
            TimerAction(period=2.0, actions=[graspnet]),
        ]
    )
