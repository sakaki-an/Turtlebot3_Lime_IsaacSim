#!/usr/bin/env bash
# Stop every tutorial script (python isaac/NN_*.py) that is still running, e.g. after an interrupted test.
# Two simulations at once both publish /clock and confuse every ROS node ("Detected jump back in time").
# "[p]ython" matches "python" but not this command line itself.
pkill -f "[p]ython isaac/[0-9]" && echo "[stop_isaac] stopped" || echo "[stop_isaac] nothing running"
