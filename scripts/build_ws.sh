#!/usr/bin/env bash
# Build the ROS 2 workspace (ros2_ws/) inside the ROS 2 Jazzy container.
#
#   ./scripts/build_ws.sh                  # build everything
#   ./scripts/build_ws.sh lime_isaac_bringup  # build selected packages
#
# The result (ros2_ws/install) is sourced automatically by every ./docker/run_ros.sh shell.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SELECT=""
[ $# -gt 0 ] && SELECT="--packages-select $*"
"$REPO_ROOT/docker/run_ros.sh" bash -c "cd /lime/ros2_ws && colcon build --symlink-install $SELECT"
