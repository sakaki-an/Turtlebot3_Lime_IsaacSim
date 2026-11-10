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
  timeout "$timeout_s" python "$script" --headless --test $(record_flag "$media") >"$log" 2>&1
  if grep -q "^\[$id\]\[test\].*PASS" "$log"; then RESULT[$id]=PASS; else RESULT[$id]=FAIL; fi
  grep "^\[$id\]" "$log" | tail -4 | sed 's/^/    /'
}

# Check that needs ROS 2 traffic: start Isaac Sim, wait for "[NN] ready", run a command in the container.
check_with_ros() {
  local id=$1 script=$2 media=$3 ros_cmd=$4 timeout_s=${5:-600}
  want "$id" || return 0
  local log="$LOG_DIR/$id.log"
  echo "[$id] $script + ROS: $ros_cmd"
  # shellcheck disable=SC2046
  ( timeout "$timeout_s" python "$script" --headless --test $(record_flag "$media") >"$log" 2>&1 ) &
  local pid=$!
  for _ in $(seq 1 150); do grep -q "^\[$id\] ready" "$log" 2>/dev/null && break; sleep 2; done
  "$ROS" bash -c "$ros_cmd" >"$LOG_DIR/$id.ros.log" 2>&1
  wait $pid
  if grep -q "^\[$id\]\[test\].*PASS" "$log"; then RESULT[$id]=PASS; else RESULT[$id]=FAIL; fi
  grep "^\[$id\]" "$log" | tail -4 | sed 's/^/    /'
}

"$ROS" true >/dev/null  # make sure the ROS container is running

check 00 isaac/00_check_setup.py "" 300
check 02 isaac/02_import_urdf.py tutorial02_import 1200
check 03 isaac/03_tune_robot.py tutorial03_tuned
check_with_ros 06 isaac/06_ros2_drive.py tutorial06_ros2_drive \
  "ros2 topic pub --times 120 -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.25}, angular: {z: 0.5}}'"

echo
echo "=== Summary ==="
fail=0
for id in $(printf '%s\n' "${!RESULT[@]}" | sort); do
  printf '  %s  %s\n' "$id" "${RESULT[$id]}"
  [ "${RESULT[$id]}" = PASS ] || fail=1
done
exit $fail
