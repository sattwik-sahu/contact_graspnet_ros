#!/usr/bin/env bash
# Pixi activation hook for the Contact-GraspNet ROS 2 workspace.
# This file is sourced, not executed.

export CONTACT_GRASPNET_ROOT="${PIXI_PROJECT_ROOT:-$(pwd)}"
export GRASPNET_CONFIG_DIR="${GRASPNET_CONFIG_DIR:-${CONTACT_GRASPNET_ROOT}/config}"
export PYTHONPATH="${CONTACT_GRASPNET_ROOT}/third_party/contact_graspnet_pytorch:${PYTHONPATH:-}"

if [ -n "${ROS_DISTRO:-}" ] && [ -f "${CONTACT_GRASPNET_ROOT}/install/${ROS_DISTRO}/setup.bash" ]; then
  # shellcheck disable=SC1090
  source "${CONTACT_GRASPNET_ROOT}/install/${ROS_DISTRO}/setup.bash"
fi
