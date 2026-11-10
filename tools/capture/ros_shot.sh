#!/usr/bin/env bash
# Screenshot of a ROS GUI (RViz, rqt) running in the container next to a tutorial script.
# Not part of the tutorials - this is how the RViz pictures on the website are made.
#
#   tools/capture/ros_shot.sh NAME "isaac/12_moveit.py" "<commands in the container>" [crop: x0 y0 x1 y1]
#
# 1. stops leftover simulations / ROS processes  2. starts the Isaac Sim script (headless)
# 3. waits for its "[NN] ready" line  4. runs the commands in the container (they must start the GUI
#    in the background and leave it running)  5. screenshots DISPLAY (default :10)  6. cleans up.
set -u
NAME=$1; SCRIPT=$2; ROS_CMDS=$3; CROP=${4:-}
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"
export DISPLAY="${DISPLAY:-:10}"
ID=$(basename "$SCRIPT" | cut -c1-2)
LOG="$REPO_ROOT/_out/shots/$NAME.isaac.log"; mkdir -p "$REPO_ROOT/_out/shots"

tools/stop_isaac.sh >/dev/null; docker/ros_clean.sh >/dev/null
( source scripts/isaac_env.sh >/dev/null && exec timeout 600 python $SCRIPT --headless >"$LOG" 2>&1 ) &
ISAAC=$!
for _ in $(seq 1 150); do grep -q "^\[$ID\] ready" "$LOG" 2>/dev/null && break; sleep 2; done
grep -q "^\[$ID\] ready" "$LOG" || { echo "[ros_shot] $SCRIPT did not become ready"; kill $ISAAC; exit 1; }

docker/run_ros.sh bash -c "$ROS_CMDS" 2>&1 | tail -5
docker exec -e DISPLAY="$DISPLAY" lime_ros import -window root "/lime/_out/shots/$NAME.full.png"

docker/ros_clean.sh >/dev/null; kill $ISAAC 2>/dev/null; tools/stop_isaac.sh >/dev/null
if [ -n "$CROP" ]; then
  "$REPO_ROOT/.venv/bin/python" - "$REPO_ROOT/_out/shots/$NAME.full.png" "$REPO_ROOT/docs/assets/img/$NAME.png" $CROP <<'PY'
import sys
from PIL import Image
src, dst, *box = sys.argv[1:]
Image.open(src).crop(tuple(int(v) for v in box)).save(dst)
print("[ros_shot] saved", dst)
PY
fi
