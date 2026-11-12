"""The "lime_room": a small room for the sensor, YOLO and Nav2 tutorials (08-13).

    6 m x 5 m floor (x: -2..4, y: -2.5..2.5), 0.6 m walls, a few obstacles, and everyday objects from
    NVIDIA's asset library on the floor in front of the robot's start pose (0, 0), so the low
    RealSense camera (5.8 cm above the floor) sees them:

        cup (025_mug), banana (011_banana), bowl (024_bowl), bottle (006_mustard_bottle)

The walls and obstacles are simple boxes: they are easy to see for the LiDAR, have colliders for the
physics, and give a clean occupancy map for Nav2 (tutorial 11).

    build_lime_room(stage)          # adds /World/LimeRoom to the current stage
    build_lime_room(stage, object_physics=True)   # ... with real objects: colliders and mass (tutorial 13)
    build_lime_room(stage, with_objects=False)    # ... with the floor cleared (tutorials 11 and 14)
    add_traffic_cone(stage)         # tutorial 11: an obstacle that is not in the map
    setup_tidy_room(stage)          # tutorial 16: mug and bottle out of the way, two more bananas (hidden)
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

# Tutorial 11: a traffic cone from NVIDIA's warehouse props (46 cm tall, 34 cm base), on Nav2's way to its goal
CONE_ASSET = "/Isaac/Environments/Simple_Warehouse/Props/S_TrafficCone.usd"
CONE_XY = (1.2, -0.3)

# Tutorial 16: two more bananas (x, y, yaw deg), hidden from the robot's start: behind the pillar, behind the divider
MORE_BANANAS = [
    ("Banana2", 2.70, -1.70, -60.0),
    ("Banana3", 1.60, 2.05, 10.0),
]
# ... and the mug and the bottle out of the way, towards the corners: not in the way, but still for the robot to
# find and check (each is within 2 m of a lookout)
TIDY_MOVES = {"Mug": (-1.2, -1.4), "Bottle": (3.4, 0.9)}


def build_lime_room(stage, root: str = "/World/LimeRoom", with_objects: bool = True, object_physics: bool = False):
    from isaacsim.core.experimental.objects import Cube, GroundPlane
    from pxr import Gf, UsdGeom, UsdLux, UsdPhysics

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
        for name, rel, x, y, yaw, half_height, up, _coco in OBJECTS:
            _place_ycb(f"{root}/Objects/{name}", rel, x, y, yaw, half_height, up)
        if object_physics:
            add_object_physics(stage, root)
    return root


def _place_ycb(path: str, rel: str, x: float, y: float, yaw: float, half_height: float, up: int):
    """Reference one YCB object and stand it on the floor at (x, y), turned by yaw (deg)."""
    import math

    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.storage.native import get_assets_root_path

    import isaacsim.core.experimental.utils.stage as stage_utils

    stage_utils.add_reference_to_stage(get_assets_root_path() + rel, path)
    cx, sx = math.sqrt(0.5), up * math.sqrt(0.5)  # +-90 deg about X
    cz, sz = math.cos(math.radians(yaw) / 2), math.sin(math.radians(yaw) / 2)  # yaw about Z
    # quaternion (w, x, y, z) of "first +-90 deg about X, then yaw about Z"
    XformPrim(path, reset_xform_op_properties=True).set_world_poses(
        positions=[[x, y, half_height + 0.002]], orientations=[[cz * cx, cz * sx, sz * sx, sz * cx]]
    )


# --8<-- [start:add_traffic_cone]
def add_traffic_cone(stage, x: float = CONE_XY[0], y: float = CONE_XY[1], root: str = "/World/LimeRoom") -> str:
    """Tutorial 11: put a traffic cone on the floor AFTER the map was made, so it is not in the map.
    It is a rigid body (1 kg): if the robot touches it, it moves. The asset already has a mesh collider."""
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.storage.native import get_assets_root_path
    from pxr import Usd, UsdGeom, UsdPhysics

    import isaacsim.core.experimental.utils.stage as stage_utils

    path = f"{root}/Cone"
    stage_utils.add_reference_to_stage(get_assets_root_path() + CONE_ASSET, path)
    XformPrim(path, reset_xform_op_properties=True).set_world_poses(positions=[[x, y, 0.002]])
    prim = stage.GetPrimAtPath(path)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(1.0)  # kg
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):  # a moving body needs a convex collider (a cone is convex anyway)
            UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexHull")
    return path
# --8<-- [end:add_traffic_cone]


# --8<-- [start:add_object_physics]
def add_object_physics(stage, root: str = "/World/LimeRoom"):
    """Make the objects real: colliders and mass, so the robot cannot drive through them (tutorial 13).

    Without this (tutorials 08-10) they are only pictures for the camera. With it:
      * banana, mug, bottle: rigid bodies with their real (YCB) masses: they fall, can be pushed, knocked over
        and lifted. Colliders: a convex decomposition for the banana (a single convex hull would make the
        curved banana a 7 cm wide blob, too wide for the gripper) and for the mug (with a convex hull it crept
        across the floor by itself); a convex hull for the bottle.
      * bowl: a static collider (it never moves) that keeps its exact, hollow shape ("none" = the triangle mesh
        itself, which PhysX allows for static objects only)
    """
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics

    def colliders(prim, approximation):
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(p)
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr(approximation)

    for name, mass, approximation in (
        ("Banana", 0.066, "convexDecomposition"),
        ("Mug", 0.118, "convexDecomposition"),
        ("Bottle", 0.603, "convexHull"),
    ):
        prim = stage.GetPrimAtPath(f"{root}/Objects/{name}")
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(mass)  # kg
        colliders(prim, approximation)
    # a banana squeezed between two fingers (tutorial 13) needs more solver iterations
    banana = stage.GetPrimAtPath(f"{root}/Objects/Banana")
    PhysxSchema.PhysxRigidBodyAPI.Apply(banana).CreateSolverPositionIterationCountAttr(32)
    colliders(stage.GetPrimAtPath(f"{root}/Objects/Bowl"), "none")
# --8<-- [end:add_object_physics]


# --8<-- [start:setup_tidy_room]
def setup_tidy_room(stage, root: str = "/World/LimeRoom") -> list[str]:
    """Tutorial 16: move the mug and the bottle out of the way (TIDY_MOVES), and add two more bananas, real
    ones like the first (66 g, convex decomposition, rubber-like friction). Call it after setup_pick_and_place(),
    which makes the rubber material. Returns the paths of all three bananas."""
    from isaacsim.core.experimental.prims import XformPrim
    from pxr import PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    for name, (x, y) in TIDY_MOVES.items():
        prim = XformPrim(f"{root}/Objects/{name}")
        position, orientation = prim.get_world_poses()
        prim.set_world_poses(positions=[[x, y, float(position.numpy()[0, 2])]], orientations=orientation)

    _name, rel, _x, _y, _yaw, half_height, up, _coco = next(o for o in OBJECTS if o[0] == "Banana")
    rubber = UsdShade.Material(stage.GetPrimAtPath(f"{root}/PhysicsMaterials/rubber"))
    paths = [f"{root}/Objects/Banana"]
    for name, x, y, yaw in MORE_BANANAS:
        path = f"{root}/Objects/{name}"
        _place_ycb(path, rel, x, y, yaw, half_height, up)
        prim = stage.GetPrimAtPath(path)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.066)  # kg
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(p)
                UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr("convexDecomposition")
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSolverPositionIterationCountAttr(32)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(rubber, UsdShade.Tokens.weakerThanDescendants, "physics")
        paths.append(path)
    return paths
# --8<-- [end:setup_tidy_room]


# --8<-- [start:setup_pick_and_place]
BOWL_SCALE = 1.5  # tutorial 13: the 19 cm banana fits into a 24 cm bowl, not into the 16 cm original


def setup_pick_and_place(stage, robot_path: str, root: str = "/World/LimeRoom"):
    """Tutorial 13, on top of add_object_physics(): a bowl the banana fits into, and grippy fingers.
      * bowl: 1.5x bigger (its static collider scales with it)
      * fingers and banana: rubber-like friction 1.0, so a 10 N squeeze holds the banana
    """
    from isaacsim.core.experimental.prims import XformPrim
    from pxr import Usd, UsdPhysics, UsdShade

    _name, _asset, x, y, _yaw, half_height, _up, _coco = next(o for o in OBJECTS if o[0] == "Bowl")
    bowl = XformPrim(f"{root}/Objects/Bowl")
    bowl.set_local_scales([[BOWL_SCALE] * 3])
    _position, orientation = bowl.get_world_poses()
    bowl.set_world_poses(positions=[[x, y, BOWL_SCALE * half_height + 0.002]], orientations=orientation)

    rubber = UsdShade.Material.Define(stage, f"{root}/PhysicsMaterials/rubber")
    api = UsdPhysics.MaterialAPI.Apply(rubber.GetPrim())
    api.CreateStaticFrictionAttr(1.0)
    api.CreateDynamicFrictionAttr(1.0)
    banana = stage.GetPrimAtPath(f"{root}/Objects/Banana")
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
