"""Sensors of the TurtleBot3 Lime, created in Isaac Sim at the frames of the URDF (tutorials 08 and 09).

    add_d435_camera(robot)  RealSense D435 color camera  -> camera_color_optical_frame
    add_lds_lidar(robot)    LDS-01 2D LiDAR (RTX)        -> base_scan
    add_imu(robot)          IMU (physics)                -> base_link (at imu_link)

All values come from the Lime's URDF/xacro (turtlebot3_bigwheel_orin.gazebo.xacro):
  camera: 640 x 480, horizontal FOV 1.211 rad (69.4 deg)
  LDS-01: 360 samples / revolution, 5 Hz, range 0.12 ... 3.5 m
"""

import math

from . import lime

# RealSense D435 color stream (URDF: <horizontal_fov>1.211</horizontal_fov>, 640 x 480)
CAMERA_WIDTH, CAMERA_HEIGHT = 640, 480
CAMERA_HFOV = 1.211  # rad
# LDS-01 (URDF: 360 samples, 5 Hz, min 0.12 m, max 3.5 m)
LIDAR_HZ, LIDAR_SAMPLES, LIDAR_MIN, LIDAR_MAX = 5.0, 360, 0.12, 3.5
IMU_OFFSET_Z = 0.033  # imu_link is 33 mm above base_link


# --8<-- [start:add_d435_camera]
def add_d435_camera(robot_path: str) -> str:
    """Create a USD camera at camera_color_optical_frame. Returns its prim path."""
    from pxr import Gf, UsdGeom

    import isaacsim.core.experimental.utils.stage as stage_utils

    stage = stage_utils.get_current_stage()
    path = f"{robot_path}/{lime.CAMERA_COLOR_OPTICAL_FRAME}/d435_color"
    camera = UsdGeom.Camera.Define(stage, path)
    # A USD camera looks along its -Z axis with +Y up. A ROS *optical* frame looks along +Z with +Y down.
    # Rotating 180 deg about X turns one into the other.
    UsdGeom.Xformable(camera.GetPrim()).AddRotateXOp().Set(180.0)
    # Pinhole model: focal length / aperture (in mm) give the field of view.
    h_aperture = 20.955  # mm (default USD camera aperture)
    camera.CreateHorizontalApertureAttr(h_aperture)
    camera.CreateVerticalApertureAttr(h_aperture * CAMERA_HEIGHT / CAMERA_WIDTH)
    camera.CreateFocalLengthAttr(h_aperture / (2.0 * math.tan(CAMERA_HFOV / 2.0)))  # = 15.1 mm
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
    return path
# --8<-- [end:add_d435_camera]


# --8<-- [start:add_lds_lidar]
def add_lds_lidar(robot_path: str) -> str:
    """Create an RTX LiDAR at base_scan that behaves like the Lime's LDS-01. Returns the OmniLidar prim path."""
    from pxr import UsdGeom

    from isaacsim.sensors.experimental.rtx import Lidar

    firing_hz = LIDAR_HZ * LIDAR_SAMPLES  # one beam fired 1800 times/s while turning at 5 Hz -> 360 points/turn
    lidar = Lidar.create(
        f"{robot_path}/{lime.LIDAR_FRAME}/lds",
        config="RPLIDAR_S2E",  # a single-beam 2D spinning LiDAR from Isaac Sim's sensor library ...
        tick_rate=LIDAR_HZ,
        translations=[[0.0, 0.0, 0.0]],
        attributes={  # ... re-configured to the LDS-01's numbers
            "omni:sensor:Core:scanRateBaseHz": int(LIDAR_HZ),
            "omni:sensor:Core:patternFiringRateHz": int(firing_hz),
            "omni:sensor:Core:reportRateBaseHz": int(firing_hz),
            "omni:sensor:Core:nearRangeM": LIDAR_MIN,
            "omni:sensor:Core:farRangeM": LIDAR_MAX,
        },
    )
    # The Lime already has its own LDS mesh: hide the RPLIDAR housing meshes that come with the asset.
    from pxr import Usd

    from isaacsim.core.experimental.utils.stage import get_current_stage

    for prim in Usd.PrimRange(get_current_stage().GetPrimAtPath(f"{robot_path}/{lime.LIDAR_FRAME}/lds")):
        if prim.IsA(UsdGeom.Gprim):
            UsdGeom.Imageable(prim).MakeInvisible()
    return lidar.paths[0]
# --8<-- [end:add_lds_lidar]


# --8<-- [start:add_imu]
def add_imu(robot_path: str) -> str:
    """Create a physics IMU on base_link, at the position of imu_link. Returns its prim path."""
    from isaacsim.sensors.experimental.physics import IMU

    # The IMU must sit on a rigid body. imu_link is only a frame (no mass), so attach the sensor to
    # base_link and move it to where imu_link is.
    imu = IMU.create(f"{robot_path}/{lime.BASE_LINK}/imu", translations=[[0.0, 0.0, IMU_OFFSET_Z]])
    return imu.paths[0]
# --8<-- [end:add_imu]
