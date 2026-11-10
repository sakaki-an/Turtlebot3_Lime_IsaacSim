#!/usr/bin/env bash
# Start (or attach to) the ROS 2 Jazzy container used by the tutorials.
#
#   ./docker/run_ros.sh                 # interactive shell (first call starts the container)
#   ./docker/run_ros.sh ros2 topic list # run one command inside the container
#   ./docker/run_ros.sh bash -c 'a && b' # shell syntax
#
# The container uses the host network, so ROS 2 nodes inside it see Isaac Sim running on the host.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LIME_ROS_IMAGE:-lime_ros_jazzy:latest}"
NAME="${LIME_ROS_CONTAINER:-lime_ros}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  echo "[run_ros] Building $IMAGE (first time only)..."
  docker build ${UBUNTU_MIRROR:+--build-arg UBUNTU_MIRROR=$UBUNTU_MIRROR} -t "$IMAGE" "$REPO_ROOT/docker/ros_jazzy"
fi

# Allow GUI apps (RViz) from the container on your X display.
if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$NAME"; then
  # Run as *your* user so files written into the repo (colcon build/, maps, ...) are owned by you.
  docker run -d --rm --name "$NAME" \
    --user "$(id -u):$(id -g)" -e HOME=/tmp/lime_home \
    --network host --ipc host \
    -e DISPLAY="${DISPLAY:-}" \
    -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$REPO_ROOT":/lime:rw \
    "$IMAGE" sleep infinity >/dev/null
  echo "[run_ros] Started container '$NAME' (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
fi

if [ $# -eq 0 ]; then
  exec docker exec -it -e DISPLAY="${DISPLAY:-}" "$NAME" /entrypoint.sh bash
else
  # Run one command in a sourced ROS environment. Arguments are passed through unchanged, so
  # quoting works as usual:  ./docker/run_ros.sh ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{...}"
  # For shell syntax (pipes, ;, &&) wrap it:  ./docker/run_ros.sh bash -c 'cmd1 && cmd2'
  TTY_FLAG=""; [ -t 0 ] && TTY_FLAG="-it"
  exec docker exec $TTY_FLAG -e DISPLAY="${DISPLAY:-}" "$NAME" /entrypoint.sh "$@"
fi
