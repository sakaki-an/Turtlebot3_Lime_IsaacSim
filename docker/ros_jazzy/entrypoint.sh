#!/bin/bash
# Source ROS 2 Jazzy and (if built) the tutorial workspace, then run the given command.
set -e
mkdir -p "$HOME"
source /opt/ros/jazzy/setup.bash
if [ -f /lime/ros2_ws/install/setup.bash ]; then
  source /lime/ros2_ws/install/setup.bash
fi
# Make every interactive shell in the container ready too.
grep -q "lime_ros_env" ~/.bashrc 2>/dev/null || cat >> ~/.bashrc <<'EOF'
# lime_ros_env
source /opt/ros/jazzy/setup.bash
[ -f /lime/ros2_ws/install/setup.bash ] && source /lime/ros2_ws/install/setup.bash
EOF
exec "$@"
