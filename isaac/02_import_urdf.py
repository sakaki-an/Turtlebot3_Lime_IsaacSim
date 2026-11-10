"""Tutorial 02 - Import TurtleBot3 Lime from URDF into USD.

    python isaac/02_import_urdf.py              # import, then show the robot in the GUI
    python isaac/02_import_urdf.py --headless   # import only, no window

What it does
  1. Converts assets/urdf/turtlebot3_lime.urdf into a USD asset with the Isaac Sim 6.2
     URDF importer (``URDFImporter`` + ``URDFImporterConfig``).
  2. Opens a new stage, adds a ground plane + light, and references the new robot USD.
  3. Presses Play so you can watch the robot drop onto the ground.

The importer writes to assets/usd/turtlebot3_lime/ :
    turtlebot3_lime.usda      <- the file you open / reference (an "interface" layer)
    payloads/                 <- geometry, materials, physics (loaded through the interface layer)
"""

import shutil
import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

# --- 1. Start Isaac Sim. Nothing from omni / isaacsim / pxr may be imported before this line. ---
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from common.paths import LIME_IMPORTED_USD, LIME_URDF, USD_DIR
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

# The URDF importer is an extension; the default standalone app does not load it.
app_utils.enable_extension("isaacsim.asset.importer.urdf")
from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

# --- 2. Convert URDF -> USD -----------------------------------------------------------------------
# The importer never overwrites: an existing folder would make it write "turtlebot3_lime_1".
# This tutorial re-creates the asset from scratch each time, so remove the previous output first.
out_dir = LIME_IMPORTED_USD.parent
if out_dir.exists():
    print(f"[02] removing previous import: {out_dir}")
    shutil.rmtree(out_dir)

config = URDFImporterConfig(
    urdf_path=str(LIME_URDF),
    usd_path=str(USD_DIR),  # output *folder*; the importer adds a sub-folder named after the robot
    fix_base=False,  # a mobile robot must NOT be welded to the world
    merge_fixed_joints=False,  # keep sensor frames (base_scan, camera_*_optical_frame, imu_link) as prims
    collision_from_visuals=False,  # the URDF already has good collision shapes
    robot_type="Mobile Manipulators",
)
usd_path = URDFImporter(config).import_urdf()
print(f"[02] URDF imported -> {usd_path}")

# Housekeeping: the importer records the absolute path of its temporary files in a comment
# ("doc") of payloads/base.usda. Clear it so the asset does not depend on (or reveal) this machine.
from pxr import Sdf

base_layer = Sdf.Layer.FindOrOpen(str(out_dir / "payloads" / "base.usda"))
if base_layer is not None and base_layer.documentation:
    base_layer.documentation = ""
    base_layer.Save()

# --- 3. Build a small scene and add the robot -----------------------------------------------------
stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
# The imported asset can carry physics for several engines. It has a "Physics" variant set
# (none / physics / physx / mujoco) and NO default selection - without a selection the robot has
# no physics at all. Isaac Sim simulates with PhysX by default, so pick "physx".
stage_utils.add_reference_to_stage(usd_path, "/World/turtlebot3_lime", variants=[("Physics", "physx")])

# Lift it 5 cm so it drops onto the ground when the simulation starts.
# reset_xform_op_properties=True gives the prim a standard translate/orient/scale transform first.
from isaacsim.core.experimental.prims import XformPrim

XformPrim("/World/turtlebot3_lime", reset_xform_op_properties=True).set_world_poses(positions=[0.0, 0.0, 0.05])

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(0.9, -0.9, 0.6), target=(0.0, 0.0, 0.2))

# --- 4. Play ----------------------------------------------------------------------------------------
SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()

# The importer marked the robot root as an articulation; find it and list its joints.
from pxr import UsdPhysics

root = next(
    p.GetPath().pathString
    for p in stage.Traverse()
    if p.HasAPI(UsdPhysics.ArticulationRootAPI) and p.GetPath().pathString.startswith("/World/turtlebot3_lime")
)
robot = Articulation(root)
print(f"[02] articulation root: {root}")
print(f"[02] {robot.num_dofs} degrees of freedom: {robot.dof_names}")

steps = 0
while simulation_app.is_running():
    simulation_app.update()
    if recorder:
        recorder.capture()
    steps += 1
    if (args.test or args.record) and steps >= 180:  # 3 s of simulated time
        break

ok = True
if args.test:
    expected = {"wheel_left_joint", "wheel_right_joint", *[f"joint{i}" for i in range(1, 7)], "gripper_left_joint"}
    missing = expected - set(robot.dof_names)
    z = float(robot.get_world_poses()[0].numpy()[0, 2])
    print(f"[02][test] missing joints: {missing or 'none'}; base height after 3 s: {z:.3f} m")
    ok = not missing and -0.02 < z < 0.2
    print(f"[02][test] {'PASS' if ok else 'FAIL'}")

if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
