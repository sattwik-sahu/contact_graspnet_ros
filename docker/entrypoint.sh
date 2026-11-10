#!/bin/bash
set -e

# shellcheck disable=SC1091
source /shell-hook
# shellcheck disable=SC1091
source "/opt/contact_graspnet_ros/install/${ROS_DISTRO}/setup.bash"

exec "$@"
