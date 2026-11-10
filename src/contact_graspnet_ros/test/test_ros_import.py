"""Smoke test for ROS-side imports.

This test is skipped unless a ROS 2 environment with the generated messages is
sourced.
"""

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("contact_graspnet_msgs")


def test_node_module_imports() -> None:
    from contact_graspnet_ros import node

    assert hasattr(node, "ContactGraspNetNode")
    assert hasattr(node, "main")
