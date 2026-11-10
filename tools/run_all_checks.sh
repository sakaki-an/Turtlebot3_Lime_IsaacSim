#!/usr/bin/env bash
# Run the automatic check of every tutorial, the same way the docs tell you to run them.
#
#   tools/run_all_checks.sh                # all checks
#   tools/run_all_checks.sh 03 06          # only some tutorials
#   RECORD=1 tools/run_all_checks.sh       # also re-create the videos / images in docs/assets
#
# A check passes only if the script prints its "[NN][test] ... PASS" line (a crashing script can still
# exit with code 0, so the exit code alone is not trusted). Logs go to _out/checks/.
set -u
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"
# shellcheck disable=SC1091
source scripts/isaac_env.sh >/dev/null
LOG_DIR="$REPO_ROOT/_out/checks"
mkdir -p "$LOG_DIR"
ROS="$REPO_ROOT/docker/run_ros.sh"
declare -A RESULT

want() {  # is tutorial $1 selected? (no arguments = all)
  [ ${#SELECTED[@]} -eq 0 ] && return 0
  local s; for s in "${SELECTED[@]}"; do [ "$s" = "$1" ] && return 0; done
  return 1
}
SELECTED=("$@")

record_flag() { [ "${RECORD:-0}" = "1" ] && [ -n "$1" ] && echo "--record $1"; }

# Isaac-only check: run the script headless with --test.
check() {
  local id=$1 script=$2 media=$3 timeout_s=${4:-900}
  want "$id" || return 0
  local log="$LOG_DIR/$id.log"
  echo "[$id] $script ..."
  # shellcheck disable=SC2046
  # shellcheck disable=SC2086
  timeout "$timeout_s" python $script --headless --test $(record_flag "$media") >"$log" 2>&1
  if grep -q "^\[$id\]\[test\].*PASS" "$log"; then RESULT[$id]=PASS; else RESULT[$id]=FAIL; fi
  grep "^\[$id\]" "$log" | tail -4 | sed 's/^/    /'
}

# Check that needs ROS 2 traffic: start Isaac Sim, wait for "[NN] ready", run a command in the container.
check_with_ros() {
  local id=$1 script=$2 media=$3 ros_cmd=$4 timeout_s=${5:-600}
  want "$id" || return 0
  tools/stop_isaac.sh >/dev/null; docker/ros_clean.sh >/dev/null  # one simulation, no leftover nodes
  local log="$LOG_DIR/$id.log"
  echo "[$id] $script + ROS: $ros_cmd"
  # shellcheck disable=SC2046
  ( timeout "$timeout_s" python "$script" --headless --test $(record_flag "$media") >"$log" 2>&1 ) &
  local pid=$!
  for _ in $(seq 1 150); do grep -q "^\[$id\] ready" "$log" 2>/dev/null && break; sleep 2; done
  "$ROS" bash -c "$ros_cmd" >"$LOG_DIR/$id.ros.log" 2>&1
  wait $pid
  docker/ros_clean.sh >/dev/null
  if grep -q "^\[$id\]\[test\].*PASS" "$log"; then RESULT[$id]=PASS; else RESULT[$id]=FAIL; fi
  grep "^\[$id\]" "$log" | tail -4 | sed 's/^/    /'
}

"$ROS" true >/dev/null  # make sure the ROS container is running

check 00 isaac/00_check_setup.py "" 300
check 01 isaac/01_hello_world.py tutorial01_hello_world
# 02 imports into _out/ so the committed asset in assets/usd/ (and git history) stays untouched.
check 02 "isaac/02_import_urdf.py --out _out/import_check" tutorial02_import 1200
check 03 isaac/03_tune_robot.py tutorial03_tuned
check 04 isaac/04_hello_robot.py tutorial04_hello_robot
check 05 isaac/05_action_graph.py tutorial05_action_graph
check_with_ros 06 isaac/06_ros2_drive.py tutorial06_ros2_drive \
  "ros2 topic pub --times 120 -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.25}, angular: {z: 0.5}}'"

check 07 isaac/07_odom_tf.py tutorial07_odom_tf

# 07 (ROS side): robot_state_publisher in the container must complete the TF chain odom -> end_effector_link.
if want 07ros; then
  echo "[07ros] isaac/07_odom_tf.py + robot_state_publisher + tf2_echo"
  ( timeout 120 python isaac/07_odom_tf.py --headless >"$LOG_DIR/07ros.log" 2>&1 ) &
  for _ in $(seq 1 90); do grep -q "^\[07\] ready" "$LOG_DIR/07ros.log" 2>/dev/null && break; sleep 2; done
  "$ROS" bash -c 'ros2 launch lime_isaac_bringup robot_state_publisher.launch.py > /tmp/lime_home/rsp.log 2>&1 & RSP=$!; sleep 8;
    timeout 8 ros2 run tf2_ros tf2_echo odom end_effector_link 2>&1 | grep -m1 Translation; kill $RSP' >"$LOG_DIR/07ros.ros.log" 2>&1
  wait
  if grep -q "Translation" "$LOG_DIR/07ros.ros.log"; then RESULT[07ros]=PASS; else RESULT[07ros]=FAIL; fi
  sed 's/^/    /' "$LOG_DIR/07ros.ros.log" | head -3
fi

check 08 isaac/08_camera.py tutorial08_camera
check 09 isaac/09_lidar_imu.py tutorial09_lidar_imu

check_with_ros 10 isaac/10_yolo.py "" \
  'ros2 launch lime_yolo yolo.launch.py > /tmp/lime_home/yolo.log 2>&1 & sleep 60'

check 11 isaac/11_make_map.py tutorial11_map 600
# the map check prints [11][test] too: file its result under "11map" (only if it ran)
if [ -n "${RESULT[11]:-}" ]; then RESULT[11map]=${RESULT[11]}; unset 'RESULT[11]'; fi
check_with_ros 11 isaac/11_nav2.py tutorial11_nav2 \
  'ros2 launch lime_isaac_bringup navigation.launch.py rviz:=false > /tmp/lime_home/nav2.log 2>&1 &
   for i in $(seq 1 90); do grep -a -q "lifecycle_manager_navigation.*Managed nodes are active" /tmp/lime_home/nav2.log && break; sleep 1; done; sleep 5
   timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 3.2, y: -1.8}, orientation: {w: 1.0}}}}"'

check_with_ros 12 isaac/12_moveit.py tutorial12_moveit \
  'ros2 launch lime_isaac_moveit moveit.launch.py rviz:=false > /tmp/lime_home/moveit.log 2>&1 &
   for i in $(seq 1 90); do grep -a -q "You can start planning now" /tmp/lime_home/moveit.log && break; sleep 1; done; sleep 3
   timeout 120 ros2 run lime_isaac_moveit arm_demo --pose demo --gripper open'

check_with_ros 13 isaac/13_all_together.py tutorial13_all_together \
  'ros2 launch lime_isaac_bringup lime_full.launch.py rviz:=false > /tmp/lime_home/full.log 2>&1 &
   for i in $(seq 1 150); do grep -a -q "lifecycle_manager_navigation.*Managed nodes are active" /tmp/lime_home/full.log && grep -a -q "You can start planning now" /tmp/lime_home/full.log && break; sleep 1; done; sleep 5
   timeout 400 ros2 run lime_isaac_bringup mission' 900

echo
echo "=== Summary ==="
fail=0
for id in $(printf '%s\n' "${!RESULT[@]}" | sort); do
  printf '  %s  %s\n' "$id" "${RESULT[$id]}"
  [ "${RESULT[$id]}" = PASS ] || fail=1
done
exit $fail
