"""Tutorial 03 - Tune TurtleBot3 Lime: joint drives, wheel/caster friction, start pose.

    python isaac/03_tune_robot.py              # write the tuned asset, then try it in the GUI
    python isaac/03_tune_robot.py --headless --test

Right after the URDF import (tutorial 02) the robot is *not* ready to simulate:
  * every joint drive has stiffness = damping = 0, so the arm collapses under gravity and the
    wheels cannot be driven;
  * the two small caster pads use the default friction, so they drag on the floor when turning;
  * the arm starts stretched out at 0 rad instead of the folded "home" pose.

This script writes the fixes into a NEW layer, assets/usd/turtlebot3_lime_tuned.usda, that *references*
the imported asset. The imported files stay untouched, so re-running tutorial 02 never loses the tuning.
Then it loads the tuned robot and checks that the arm holds its pose, the wheels follow a velocity
command and the gripper opens.
"""

import math
import os
import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import lime
from common.paths import LIME_IMPORTED_USD, LIME_USD
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

# ---------------------------------------------------------------------------------------------------
# Tuning values - SI units (radians, metres, N, N*m). Change them and re-run to see the effect.
# ---------------------------------------------------------------------------------------------------
WHEEL_DRIVE = dict(stiffness=0.0, damping=100.0, max_force=3.0)  # velocity control: stiffness MUST be 0
ARM_DRIVE = dict(stiffness=200.0, damping=10.0, max_force=4.1)  # position control (XM430 stall torque 4.1 N*m)
GRIPPER_DRIVE = dict(stiffness=1000.0, damping=20.0, max_force=10.0)  # position control, prismatic (N/m)
WHEEL_FRICTION = 1.0  # rubber tyres
CASTER_FRICTION = 0.0  # the caster pads should slide freely
SOLVER_ITERATIONS = (64, 4)  # (position, velocity) iterations; PhysX TGS wants <= 4 velocity iterations


def angular_gain_to_usd(gain_per_rad: float) -> float:
    """USD stores angular drive gains per DEGREE (N*m/deg); we think in N*m/rad."""
    return gain_per_rad * math.pi / 180.0


# ---------------------------------------------------------------------------------------------------
# 1. Create the tuned layer:   turtlebot3_lime_tuned.usda  --references-->  turtlebot3_lime/turtlebot3_lime.usda
# ---------------------------------------------------------------------------------------------------
if LIME_USD.exists():
    LIME_USD.unlink()
layer = Sdf.Layer.CreateNew(str(LIME_USD))
stage = Usd.Stage.Open(layer)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

robot = stage.DefinePrim("/turtlebot3_lime", "Xform")
stage.SetDefaultPrim(robot)
robot.GetReferences().AddReference("./" + os.path.relpath(LIME_IMPORTED_USD, LIME_USD.parent))
# Select PhysX physics by default (see tutorial 02) - users of this file no longer have to.
robot.GetVariantSet("Physics").SetVariantSelection("physx")


def joint(name: str) -> Usd.Prim:
    return stage.GetPrimAtPath(f"/turtlebot3_lime/Physics/{name}")


def set_drive(name: str, kind: str, stiffness: float, damping: float, max_force: float, target: float):
    """kind: 'angular' (revolute, target in rad) or 'linear' (prismatic, target in m)."""
    prim = joint(name)
    drive = UsdPhysics.DriveAPI.Apply(prim, kind)
    to_usd = angular_gain_to_usd if kind == "angular" else (lambda g: g)
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(to_usd(stiffness))
    drive.CreateDampingAttr(to_usd(damping))
    drive.CreateMaxForceAttr(max_force)
    if stiffness > 0:  # position drive: target + initial joint position (USD angles are in degrees)
        value = math.degrees(target) if kind == "angular" else target
        drive.CreateTargetPositionAttr(value)
        prim.CreateAttribute(f"state:{kind}:physics:position", Sdf.ValueTypeNames.Float).Set(value)
    else:  # velocity drive
        drive.CreateTargetVelocityAttr(0.0)


# 2. Joint drives ------------------------------------------------------------------------------------
for name in lime.WHEEL_JOINTS:
    set_drive(name, "angular", **WHEEL_DRIVE, target=0.0)
for name, home in zip(lime.ARM_JOINTS, lime.ARM_HOME):
    set_drive(name, "angular", **ARM_DRIVE, target=home)
set_drive(lime.GRIPPER_JOINT, "linear", **GRIPPER_DRIVE, target=0.0)
# gripper_right_joint follows gripper_left_joint through its mimic joint (NewtonMimicAPI authored by the
# importer from the URDF <mimic> tag), so it gets no drive of its own.
set_drive(lime.GRIPPER_MIMIC_JOINT, "linear", stiffness=0.0, damping=0.0, max_force=GRIPPER_DRIVE["max_force"], target=0.0)

# 3. Physics materials (friction) -----------------------------------------------------------------------
def physics_material(path: str, friction: float, combine_mode: str = "average") -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, path)
    api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(friction)
    api.CreateRestitutionAttr(0.0)
    # When two materials touch, PhysX COMBINES their friction - by default it takes the AVERAGE.
    # A 0.0-friction caster on a floor with friction > 0 still gets friction > 0 and brakes the robot
    # (measured: 0.68 m instead of 0.87 m in the drive test below). "min" makes the caster's 0.0 win.
    PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(combine_mode)
    return mat


wheel_mat = physics_material("/turtlebot3_lime/PhysicsMaterials/wheel", WHEEL_FRICTION)
caster_mat = physics_material("/turtlebot3_lime/PhysicsMaterials/caster", CASTER_FRICTION, combine_mode="min")
base = f"/turtlebot3_lime/{lime.BASE_LINK}"


def bind_physics_material(prim: Usd.Prim, mat: UsdShade.Material):
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, UsdShade.Tokens.weakerThanDescendants, "physics")


for link in ["caster_back_left_link", "caster_back_right_link"]:
    bind_physics_material(stage.GetPrimAtPath(f"{base}/{link}/box"), caster_mat)

# 3b. Wheel colliders: sphere instead of cylinder ------------------------------------------------------
# The URDF tyre is a 24 mm wide cylinder that touches the floor along a LINE. In a turn the inner and
# outer edge of that line want different speeds, the tyre scrubs, and the robot turns ~12 % slower than
# commanded. A sphere of the same radius touches the floor in one POINT, like the ideal wheel the
# differential-drive formula assumes (measured: 0.44 -> 0.50 rad/s for a 0.5 rad/s command).
for link in lime.WHEEL_JOINTS:
    link_path = f"{base}/{link.replace('_joint', '_link')}"
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath(f"{link_path}/cylinder")).CreateCollisionEnabledAttr(False)
    sphere = UsdGeom.Sphere.Define(stage, f"{link_path}/sphere_collider")
    sphere.CreateRadiusAttr(lime.WHEEL_RADIUS)
    sphere.CreatePurposeAttr(UsdGeom.Tokens.guide)  # a collision shape only: not rendered
    UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    bind_physics_material(sphere.GetPrim(), wheel_mat)

# 4. Articulation solver ---------------------------------------------------------------------------------
# More solver iterations = more accurate joints + contacts (default: 32 position iterations).
articulation = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(f"/turtlebot3_lime/{lime.ARTICULATION_ROOT}"))
articulation.CreateSolverPositionIterationCountAttr(SOLVER_ITERATIONS[0])
articulation.CreateSolverVelocityIterationCountAttr(SOLVER_ITERATIONS[1])

layer.Save()
print(f"[03] wrote tuned robot: {LIME_USD}")

# ---------------------------------------------------------------------------------------------------
# 5. Try the tuned robot
# ---------------------------------------------------------------------------------------------------
stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
root_path = lime.add_lime("/World/turtlebot3_lime", position=(0.0, 0.0, 0.0))

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(1.0, -1.0, 0.7), target=(0.2, 0.0, 0.2))

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()

robot = Articulation(root_path)
arm_idx = robot.get_dof_indices(lime.ARM_JOINTS)
wheel_idx = robot.get_dof_indices(lime.WHEEL_JOINTS)
grip_idx = robot.get_dof_indices([lime.GRIPPER_JOINT, lime.GRIPPER_MIMIC_JOINT])


def base_xy():
    return robot.get_world_poses()[0].numpy()[0, :2].copy()


results = {}
step = 0
WHEEL_SPEED = 5.0  # rad/s  -> 5 * 0.06 = 0.3 m/s
while simulation_app.is_running():
    t = step / 60.0
    if step == 120:  # t = 2 s: arm settled -> measure pose error, then drive forward
        q = robot.get_dof_positions(dof_indices=arm_idx).numpy()[0]
        results["arm_error"] = float(np.max(np.abs(q - np.array(lime.ARM_HOME))))
        results["start_xy"] = base_xy()
        robot.set_dof_velocity_targets([[WHEEL_SPEED, WHEEL_SPEED]], dof_indices=wheel_idx)
    if step == 300:  # t = 5 s: stop wheels, open the gripper
        results["distance"] = float(np.linalg.norm(base_xy() - results["start_xy"]))
        robot.set_dof_velocity_targets([[0.0, 0.0]], dof_indices=wheel_idx)
        robot.set_dof_position_targets([[lime.GRIPPER_OPEN]], dof_indices=robot.get_dof_indices([lime.GRIPPER_JOINT]))
    if step == 420:  # t = 7 s: check gripper
        results["gripper"] = robot.get_dof_positions(dof_indices=grip_idx).numpy()[0].tolist()
    simulation_app.update()
    if recorder:
        recorder.look_at(eye=(base_xy()[0] + 1.0, -1.0, 0.7), target=(base_xy()[0] + 0.1, base_xy()[1], 0.15))
        recorder.capture()
    step += 1
    if (args.test or args.record) and step > 430:
        break

ok = True
if "gripper" in results:
    expected = WHEEL_SPEED * lime.WHEEL_RADIUS * 3.0  # 3 s at 0.3 m/s = 0.9 m
    left, right = results["gripper"]
    print(f"[03] arm max error after 2 s : {results['arm_error']:.3f} rad")
    print(f"[03] distance in 3 s         : {results['distance']:.3f} m (ideal {expected:.2f} m)")
    print(f"[03] gripper left / right    : {left:.4f} / {right:.4f} m (target {lime.GRIPPER_OPEN})")
    if args.test:
        ok = (
            results["arm_error"] < 0.05
            and abs(results["distance"] - expected) < 0.2 * expected
            and abs(left - lime.GRIPPER_OPEN) < 0.003
        )
        print(f"[03][test] {'PASS' if ok else 'FAIL'}")

if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
