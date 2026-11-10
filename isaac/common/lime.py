"""TurtleBot3 Lime facts used by the tutorials (taken from the URDF / SRDF).

Everything here is plain data except ``add_lime()``, which references the tuned robot USD into the
current stage (import it only after ``SimulationApp`` is created).
"""

# --- Mobile base (differential drive) -----------------------------------------------------------
WHEEL_JOINTS = ["wheel_left_joint", "wheel_right_joint"]
WHEEL_RADIUS = 0.06  # m   (URDF: wheel collision cylinder radius, base_joint z = 0.060)
WHEEL_BASE = 0.29425  # m   (URDF: wheel joints at y = +-0.147125)
MAX_WHEEL_SPEED = 8.063  # rad/s (URDF velocity limit of the XM430-W210 wheel servos)

# --- Arm (OpenManipulator "SARA", 6 DoF) + gripper -------------------------------------------------
ARM_JOINTS = [f"joint{i}" for i in range(1, 7)]
GRIPPER_JOINT = "gripper_left_joint"  # the one you command
GRIPPER_MIMIC_JOINT = "gripper_right_joint"  # follows gripper_left_joint
GRIPPER_OPEN = 0.019  # m (upper limit)
GRIPPER_CLOSED = -0.010  # m (lower limit)

# Named arm poses from turtlebot3_lime_moveit_config/config/turtlebot3_lime.srdf (radians).
# Upstream "home" has joint3 = 1.87, just beyond the URDF limit 1.8567 - MoveIt refuses to plan from a start
# state outside the limits, so this project uses 1.85 (also patched in the vendored SRDF).
ARM_HOME = [0.0, -1.10, 1.85, 0.0, 0.98, 0.0]
ARM_ZERO = [0.0] * 6

# --- Prim paths inside the robot asset -------------------------------------------------------------
# The URDF importer puts links under <robot>/Geometry (nested like the URDF tree) and joints under
# <robot>/Physics. base_link carries the articulation root.
BASE_LINK = "Geometry/base_footprint/base_link"
ARTICULATION_ROOT = BASE_LINK
LIDAR_FRAME = BASE_LINK + "/base_scan"
IMU_FRAME = BASE_LINK + "/imu_link"
CAMERA_LINK = BASE_LINK + "/camera_bottom_screw_frame/camera_link"
CAMERA_COLOR_OPTICAL_FRAME = CAMERA_LINK + "/camera_color_frame/camera_color_optical_frame"


def add_lime(prim_path: str = "/World/turtlebot3_lime", position=(0.0, 0.0, 0.0), yaw_deg: float = 0.0):
    """Reference the tuned TurtleBot3 Lime USD at ``prim_path`` and place it. Returns the articulation root path."""
    import math

    import isaacsim.core.experimental.utils.stage as stage_utils
    from isaacsim.core.experimental.prims import XformPrim

    from .paths import LIME_USD

    stage_utils.add_reference_to_stage(str(LIME_USD), prim_path)
    half = math.radians(yaw_deg) / 2.0
    XformPrim(prim_path, reset_xform_op_properties=True).set_world_poses(
        positions=[list(position)], orientations=[[math.cos(half), 0.0, 0.0, math.sin(half)]]
    )
    return f"{prim_path}/{ARTICULATION_ROOT}"
