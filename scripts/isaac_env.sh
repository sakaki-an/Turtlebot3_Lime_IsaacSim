# Source this file (do not execute it):   source scripts/isaac_env.sh
#
# Prepares a shell for running Isaac Sim 6.2 next to a ROS 2 Jazzy container:
#   * activates the repo-local Isaac Sim venv (.venv)
#   * removes any ROS environment you may have sourced and points Isaac Sim at its own
#     bundled ROS 2 Jazzy libraries (exts/isaacsim.ros2.core/jazzy/lib)
#   * sets the DDS settings shared with the ROS container (docker/run_ros.sh)

_LIME_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- 1. Clear a previously sourced ROS environment -----------------------------
# Isaac Sim must load *its own* bundled ROS 2 Jazzy libraries (step 4), not a system install.
for _v in ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH \
          CMAKE_PREFIX_PATH PYTHONPATH; do
  unset "$_v"
done
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  LD_LIBRARY_PATH="$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v -E '/opt/ros/|ros2_jazzy|ros2_ws' | paste -sd: -)"
  export LD_LIBRARY_PATH
fi
PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E '/opt/ros/|ros2_jazzy|ros2_ws' | paste -sd: -)"
export PATH

# --- 2. DDS settings (must match the ROS container) -----------------------------
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="${_LIME_REPO_ROOT}/config/fastdds.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="${_LIME_REPO_ROOT}/config/fastdds.xml"

# --- 3. Isaac Sim venv ------------------------------------------------------------
export OMNI_KIT_ACCEPT_EULA=YES
if [ -f "${_LIME_REPO_ROOT}/.venv/bin/activate" ]; then
  # shellcheck disable=SC1091
  source "${_LIME_REPO_ROOT}/.venv/bin/activate"
else
  echo "[isaac_env] .venv not found - run ./scripts/setup_isaacsim.sh first" >&2
fi

# --- 4. Isaac Sim's bundled ROS 2 Jazzy libraries ---------------------------------------
# A pip install has no isaac-sim.sh launcher, so nobody puts the bundled ROS 2 libraries on
# LD_LIBRARY_PATH for us. It must be set *before* Isaac Sim starts (the dynamic linker reads it
# once at process start), otherwise the ROS 2 bridge fails with "libament_index_cpp.so: cannot
# open shared object file".
export ROS_DISTRO=jazzy
_ISAAC_ROS_LIB="$(ls -d "${_LIME_REPO_ROOT}"/.venv/lib/python3.*/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy/lib 2>/dev/null | head -1)"
if [ -n "${_ISAAC_ROS_LIB}" ]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}${_ISAAC_ROS_LIB}"
fi
export LIME_REPO_ROOT="${_LIME_REPO_ROOT}"
echo "[isaac_env] Isaac Sim env ready (ROS_DOMAIN_ID=${ROS_DOMAIN_ID}, RMW=${RMW_IMPLEMENTATION})"
