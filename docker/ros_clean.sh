#!/usr/bin/env bash
# Stop every ROS 2 process running in the tutorial container (launch files, nodes, RViz),
# e.g. before starting the next tutorial. The container itself keeps running.
NAME="${LIME_ROS_CONTAINER:-lime_ros}"
docker ps --format '{{.Names}}' | grep -qx "$NAME" || exit 0
# The [x]yz patterns match "xyz" but not this command line itself.
docker exec "$NAME" bash -c '
  pkill -INT -f "[r]os2 launch" ; sleep 2
  pkill -f "[r]os2 launch|[r]os2 run|[r]os2 topic|[r]os2 action|[r]viz2|[m]ove_group|[r]os2_control_node|[r]obot_state_publisher|[s]pawner|[c]ontroller_manager|[n]av2|[a]mcl|[m]ap_server|[l]ifecycle_manager|[c]omponent_container|[s]lam_toolbox|[l]ime_yolo" ; sleep 1
  pkill -9 -f "[r]os2 launch|[m]ove_group|[r]os2_control_node|[r]obot_state_publisher|[r]viz2|[c]omponent_container" ; true'
echo "[ros_clean] ROS processes in '$NAME' stopped"
