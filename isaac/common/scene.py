"""The "lime_room": a small room for the sensor, YOLO and Nav2 tutorials (08-13).

    6 m x 5 m floor (x: -2..4, y: -2.5..2.5), 0.6 m walls, a few obstacles, and everyday objects from
    NVIDIA's asset library on the floor in front of the robot's start pose (0, 0), so the low
    RealSense camera (5.8 cm above the floor) sees them:

        cup (025_mug), banana (011_banana), bowl (024_bowl), bottle (006_mustard_bottle)

The walls and obstacles are simple boxes: they are easy to see for the LiDAR, have colliders for the
physics, and give a clean occupancy map for Nav2 (tutorial 11).

    build_lime_room(stage)          # adds /World/LimeRoom to the current stage
    python isaac/common/scene.py    # (re)writes assets/scenes/lime_room.usda for the GUI tutorials
"""

ROOM_X = (-2.0, 4.0)
ROOM_Y = (-2.5, 2.5)
WALL_HEIGHT = 0.6
WALL_THICKNESS = 0.1

# (name, center x, y, size x, y, z)
OBSTACLES = [
    ("Pillar", 2.0, -1.2, 0.4, 0.4, 0.6),
    ("Crate", 2.6, 1.3, 0.7, 0.4, 0.4),
    ("Divider", 1.2, 1.9, 0.1, 1.2, 0.6),
]

# (prim name, asset path below the Isaac Sim assets root, x, y, yaw deg, half height m, up, COCO class for YOLO)
# The YCB "Axis_Aligned" meshes are modelled along the Y axis (some +Y up, some -Y up) and centred on their
# origin: they are rotated +-90 deg about X (Y -> Z) and lifted by half their height to stand on the floor.
YCB = "/Isaac/Props/YCB/Axis_Aligned/"
OBJECTS = [
    ("Mug", YCB + "025_mug.usd", 0.70, 0.18, -60.0, 0.041, +1, "cup"),
    ("Banana", YCB + "011_banana.usd", 0.80, -0.12, 20.0, 0.019, +1, "banana"),
    ("Bowl", YCB + "024_bowl.usd", 1.00, 0.40, 0.0, 0.028, -1, "bowl"),
    ("Bottle", YCB + "006_mustard_bottle.usd", 0.95, -0.38, 30.0, 0.096, -1, "bottle"),
]


def build_lime_room(stage, root: str = "/World/LimeRoom", with_objects: bool = True):
    import math

    from isaacsim.core.experimental.objects import Cube, GroundPlane
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.storage.native import get_assets_root_path
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

    import isaacsim.core.experimental.utils.stage as stage_utils

    UsdGeom.Xform.Define(stage, root)
    GroundPlane(f"{root}/GroundPlane")

    dome = UsdLux.DomeLight.Define(stage, f"{root}/DomeLight")
    dome.CreateIntensityAttr(600.0)
    sun = UsdLux.DistantLight.Define(stage, f"{root}/SunLight")
    sun.CreateIntensityAttr(1500.0)
    UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-45, 25, 0))

    def box(name, cx, cy, sx, sy, sz, color):
        path = f"{root}/{name}"
        Cube(path, sizes=1.0, positions=[cx, cy, sz / 2], scales=[sx, sy, sz], colors=[color])
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(path))

    (x0, x1), (y0, y1), t, h = ROOM_X, ROOM_Y, WALL_THICKNESS, WALL_HEIGHT
    wall = [0.85, 0.85, 0.82]
    box("Wall_North", (x0 + x1) / 2, y1 + t / 2, x1 - x0 + 2 * t, t, h, wall)
    box("Wall_South", (x0 + x1) / 2, y0 - t / 2, x1 - x0 + 2 * t, t, h, wall)
    box("Wall_East", x1 + t / 2, (y0 + y1) / 2, t, y1 - y0, h, wall)
    box("Wall_West", x0 - t / 2, (y0 + y1) / 2, t, y1 - y0, h, wall)
    for name, cx, cy, sx, sy, sz in OBSTACLES:
        box(name, cx, cy, sx, sy, sz, [0.95, 0.55, 0.15])

    if with_objects:
        assets = get_assets_root_path()
        for name, rel, x, y, yaw, half_height, up, _coco in OBJECTS:
            path = f"{root}/Objects/{name}"
            stage_utils.add_reference_to_stage(assets + rel, path)
            cx, sx = math.sqrt(0.5), up * math.sqrt(0.5)  # +-90 deg about X
            cz, sz = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)  # yaw about Z
            # quaternion (w, x, y, z) of "first +-90 deg about X, then yaw about Z"
            XformPrim(path, reset_xform_op_properties=True).set_world_poses(
                positions=[[x, y, half_height + 0.002]], orientations=[[cz * cx, cz * sx, sz * sx, sz * cx]]
            )
    return root


# --8<-- [start:setup_pick_and_place]
BOWL_SCALE = 1.5  # tutorial 13: the 19 cm banana fits into a 24 cm bowl, not into the 16 cm original


def setup_pick_and_place(stage, robot_path: str, root: str = "/World/LimeRoom"):
    """Tutorial 13: turn the banana into a real object the gripper can hold, and the bowl into a container.

    In tutorials 08-12 the objects are only pictures for the camera: no collider, no mass. Here:
      * banana: rigid body (it falls, can be pushed and lifted), 66 g, a convex-decomposition collider
        (a single convex hull would make the curved banana a 7 cm wide blob, too wide for the gripper)
      * bowl:   1.5x bigger, a static collider that keeps its exact (concave) shape
      * fingers and banana: rubber-like friction 1.0, so a 10 N squeeze holds the banana
    """
    from isaacsim.core.experimental.prims import XformPrim
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    def colliders(prim, approximation):
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(p)
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approximation)

    banana = stage.GetPrimAtPath(f"{root}/Objects/Banana")
    UsdPhysics.RigidBodyAPI.Apply(banana)
    UsdPhysics.MassAPI.Apply(banana).CreateMassAttr(0.066)
    body = PhysxSchema.PhysxRigidBodyAPI.Apply(banana)
    body.CreateSolverPositionIterationCountAttr(32)  # an object squeezed between two fingers needs more iterations
    colliders(banana, "convexDecomposition")

    bowl_path = f"{root}/Objects/Bowl"
    _name, _asset, x, y, _yaw, half_height, _up, _coco = next(o for o in OBJECTS if o[0] == "Bowl")
    bowl = XformPrim(bowl_path)
    bowl.set_local_scales([[BOWL_SCALE] * 3])
    position, orientation = bowl.get_world_poses()
    bowl.set_world_poses(positions=[[x, y, BOWL_SCALE * half_height + 0.002]], orientations=orientation)
    colliders(stage.GetPrimAtPath(bowl_path), "none")  # "none" = the triangle mesh itself (static objects only)

    rubber = UsdShade.Material.Define(stage, f"{root}/PhysicsMaterials/rubber")
    api = UsdPhysics.MaterialAPI.Apply(rubber.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(1.0)
    fingers = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path)) if p.GetName() in ("gripper_left_link", "gripper_right_link")]
    for prim in [banana, *fingers]:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(rubber, UsdShade.Tokens.weakerThanDescendants, "physics")
# --8<-- [end:setup_pick_and_place]


if __name__ == "__main__":
    # Write assets/scenes/lime_room.usda (used by the GUI instructions).
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    import isaacsim.core.experimental.utils.stage as stage_utils
    from common.paths import LIME_ROOM_USD

    stage = stage_utils.create_new_stage(template="empty")
    build_lime_room(stage)
    while stage_utils.is_stage_loading():
        app.update()
    LIME_ROOM_USD.parent.mkdir(parents=True, exist_ok=True)
    stage_utils.save_stage(str(LIME_ROOM_USD))
    print(f"[scene] saved {LIME_ROOM_USD}")
    app.close()
