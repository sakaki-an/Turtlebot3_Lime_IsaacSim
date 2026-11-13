"""Tutorial 03 - before and after: the robot as imported next to the tuned robot, same test.

    python isaac/03_before_after.py                     # watch it in the GUI
    python isaac/03_before_after.py --headless --test

Two Limes stand side by side and get the drive test of 03_tune_robot.py: wait 2 s, both wheels at 5 rad/s
for 3 s, then open the gripper.
  * "as imported": the asset exactly as tutorial 02 imported it, with ONE change: its wheels get the same
    velocity-drive damping as the tuned robot (set at runtime). Without it the wheels could not follow the
    speed command at all, and there would be nothing to compare.
  * "tuned": the robot from tutorial 03 (assets/usd/turtlebot3_lime_tuned.usda).
Run 03_tune_robot.py first: this script only reads the two assets, it writes none.
"""

import math
import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import lime
from common.paths import LIME_IMPORTED_USD
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import Gf, UsdGeom, UsdLux

WHEEL_SPEED = 5.0  # rad/s, as in 03_tune_robot.py: 5 * 0.06 = 0.3 m/s, 0.9 m in 3 s
WHEEL_DAMPING = 100.0  # N*m*s/rad, the tuned robot's wheel drive (03_tune_robot.py: WHEEL_DRIVE)
LANES = {"as imported": -0.4, "tuned": 0.4}  # y of each robot's lane, m (seen from the front: left, right)

stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")

# --8<-- [start:robots]
# The robot as imported (tutorial 02): pick the PhysX variant, place it. Nothing else.
stage_utils.add_reference_to_stage(str(LIME_IMPORTED_USD), "/World/as_imported", variants=[("Physics", "physx")])
XformPrim("/World/as_imported", reset_xform_op_properties=True).set_world_poses(positions=[[0.0, LANES["as imported"], 0.0]])
# The tuned robot (tutorial 03).
lime.add_lime("/World/tuned", position=(0.0, LANES["tuned"], 0.0))
# --8<-- [end:robots]


def floor_line(path, x, y, length, colour):
    """A thin painted line on the floor across a lane (looks only: no collider)."""
    line = UsdGeom.Cube.Define(stage, path)
    line.CreateSizeAttr(1.0)
    line.CreateDisplayColorAttr([Gf.Vec3f(*colour)])
    xform = UsdGeom.Xformable(line.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.001))
    xform.AddScaleOp().Set(Gf.Vec3f(0.015, length, 0.002))


for name, y in LANES.items():
    tag = name.replace(" ", "_")
    floor_line(f"/World/Lines/start_{tag}", 0.0, y, 0.5, (0.1, 0.1, 0.1))
    floor_line(f"/World/Lines/ideal_{tag}", WHEEL_SPEED * lime.WHEEL_RADIUS * 3.0, y, 0.5, (0.9, 0.5, 0.05))

recorder = None
if args.record:
    from common.recorder import Recorder

    # A fixed camera in front of the two lanes: "as imported" on the left, "tuned" on the right.
    recorder = Recorder(args.record, eye=(4.0, 0.0, 1.75), target=(0.55, 0.0, 0.12), focal_length=24.0)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()

robots = {
    "as imported": Articulation(f"/World/as_imported/{lime.ARTICULATION_ROOT}"),
    "tuned": Articulation(f"/World/tuned/{lime.ARTICULATION_ROOT}"),
}
# --8<-- [start:one_change]
# The ONE change to the imported robot: a velocity drive on the wheels (stiffness 0, damping as tuned).
# The Python API takes SI units (N*m*s/rad); it changes the running simulation only, not the asset.
before = robots["as imported"]
wheels = before.get_dof_indices(lime.WHEEL_JOINTS)
before.set_dof_gains(stiffnesses=[[0.0, 0.0]], dampings=[[WHEEL_DAMPING, WHEEL_DAMPING]], dof_indices=wheels)
# --8<-- [end:one_change]


def pose(robot):
    """x, y (m) and yaw (rad) of the robot's base."""
    p, q = robot.get_world_poses()
    w, x, y, z = q.numpy()[0]
    return float(p.numpy()[0, 0]), float(p.numpy()[0, 1]), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def arm(robot):
    return robot.get_dof_positions(dof_indices=robot.get_dof_indices(lime.ARM_JOINTS)).numpy()[0]


def gripper(robot):
    return float(robot.get_dof_positions(dof_indices=robot.get_dof_indices([lime.GRIPPER_JOINT])).numpy()[0, 0])


start_arm = {name: arm(robot) for name, robot in robots.items()}
frames = []  # what the overlay prints on each recorded frame
results = {name: {"wheel_speeds": []} for name in robots}
step = 0
while simulation_app.is_running():
    for name, robot in robots.items():
        r = results[name]
        wheel_idx = robot.get_dof_indices(lime.WHEEL_JOINTS)
        if step == 120:  # t = 2 s: how far did the arm move by itself? Then drive.
            r["arm_moved"] = float(np.max(np.abs(arm(robot) - start_arm[name])))
            r["start"] = pose(robot)
            robot.set_dof_velocity_targets([[WHEEL_SPEED, WHEEL_SPEED]], dof_indices=wheel_idx)
        if 210 <= step < 300:  # the last 1.5 s of the drive: wheel speeds
            r["wheel_speeds"].append(robot.get_dof_velocities(dof_indices=wheel_idx).numpy()[0])
        if step == 300:  # t = 5 s: stop, open the gripper
            x0, y0, yaw0 = r["start"]
            x, y, yaw = pose(robot)
            r["distance"], r["drift"], r["turn"] = math.hypot(x - x0, y - y0), y - y0, math.degrees(yaw - yaw0)
            robot.set_dof_velocity_targets([[0.0, 0.0]], dof_indices=wheel_idx)
            robot.set_dof_position_targets([[lime.GRIPPER_OPEN]], dof_indices=robot.get_dof_indices([lime.GRIPPER_JOINT]))
        if step == 420:  # t = 7 s
            r["gripper"] = gripper(robot)
    simulation_app.update()
    if recorder:
        n = len(recorder.frames)
        recorder.capture()
        if len(recorder.frames) > n:  # a frame was recorded: note what to print on it
            live = {}
            for name, robot in robots.items():
                r = results[name]
                x, y, yaw = pose(robot)
                x0, y0, yaw0 = r.get("start", (x, y, yaw))
                driven, turn = (r["distance"], r["turn"]) if "distance" in r else (math.hypot(x - x0, y - y0), math.degrees(yaw - yaw0))
                live[name] = dict(arm=float(np.max(np.abs(arm(robot) - start_arm[name]))), driven=driven, turn=turn,
                                  gripper=gripper(robot))
            frames.append((step / 60.0, live))
    step += 1
    if (args.test or args.record) and step > 430:
        break

ok = True
if all("gripper" in r for r in results.values()):
    for name, r in results.items():
        left, right = np.mean(r["wheel_speeds"], axis=0)
        print(f"[03vs] {name:12s}: arm moved {r['arm_moved']:.3f} rad in 2 s, drove {r['distance']:.3f} m "
              f"(sideways {r['drift']:+.3f} m, turned {r['turn']:+.1f} deg), wheels {left:.2f} / {right:.2f} rad/s, "
              f"gripper {r['gripper'] * 1000:.1f} mm")
    if args.test:
        ideal = WHEEL_SPEED * lime.WHEEL_RADIUS * 3.0
        tuned, imported = results["tuned"], results["as imported"]
        checks = [  # the tuned robot passes tutorial 03's test; the imported one fails where it must
            (tuned["arm_moved"] < 0.05, f"tuned: arm holds its pose (moved {tuned['arm_moved']:.3f} rad)"),
            (abs(tuned["distance"] - ideal) < 0.2 * ideal, f"tuned: drove {tuned['distance']:.2f} m (ideal {ideal:.2f} m)"),
            (abs(tuned["gripper"] - lime.GRIPPER_OPEN) < 0.003, f"tuned: gripper open ({tuned['gripper'] * 1000:.1f} mm)"),
            (imported["arm_moved"] > 0.5, f"as imported: arm falls (moved {imported['arm_moved']:.2f} rad)"),
            (imported["gripper"] < 0.005, f"as imported: gripper does not open ({imported['gripper'] * 1000:.1f} mm)"),
        ]
        for passed, text in checks:
            print(f"[03vs][check] {'ok  ' if passed else 'FAIL'} {text}")
        ok = all(passed for passed, _ in checks)
        print(f"[03vs][test] {'PASS' if ok else 'FAIL'}")
else:
    ok = False

if recorder:
    # Print the live numbers on the frames, hold the last frame for 2 s, save.
    from PIL import Image, ImageDraw, ImageFont

    big, small = ImageFont.load_default(size=28), ImageFont.load_default(size=22)
    TITLES = {"as imported": "As imported (+ wheel drive)", "tuned": "Tuned"}

    def phase(t):
        if t < 2.0:
            return f"t = {t:.1f} s   waiting: does the arm hold its pose?"
        if t < 5.0:
            return f"t = {t:.1f} s   both wheels 5 rad/s for 3 s (ideal 0.90 m)"
        return f"t = {t:.1f} s   stop, open the gripper"

    def show(value, spec):
        """format() without a "-0.0" for values that round to zero."""
        return format(float(format(value, spec)) + 0.0, spec)

    def annotate(frame, t, live):
        img = Image.fromarray(frame)
        draw = ImageDraw.Draw(img, "RGBA")
        width, height = img.size
        text = phase(t)
        w = draw.textlength(text, font=big)
        draw.rounded_rectangle(((width - w) / 2 - 16, 14, (width + w) / 2 + 16, 62), 10, fill=(0, 0, 0, 160))
        draw.text(((width - w) / 2, 22), text, font=big, fill="white")
        for i, name in enumerate(TITLES):
            v = live[name]
            lines = [("arm moved", f"{show(math.degrees(v['arm']), '.1f')} deg"), ("driven", f"{show(v['driven'], '.2f')} m"),
                     ("turned", f"{show(v['turn'], '+.1f')} deg"), ("gripper", f"{show(v['gripper'] * 1000, '.1f')} mm")]
            x0 = 24 if i == 0 else width - 24 - 380
            draw.rounded_rectangle((x0, height - 176, x0 + 380, height - 20), 10, fill=(0, 0, 0, 160))
            draw.text((x0 + 16, height - 168), TITLES[name], font=big, fill=(255, 210, 120) if i == 0 else (170, 240, 120))
            for k, (label, value) in enumerate(lines):
                draw.text((x0 + 16, height - 128 + k * 26), label, font=small, fill="white")
                draw.text((x0 + 180, height - 128 + k * 26), value, font=small, fill="white")
        return np.asarray(img)

    recorder.frames = [annotate(f, t, live) for f, (t, live) in zip(recorder.frames, frames)]
    recorder.frames += [recorder.frames[-1]] * 60
    recorder.save()

app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
