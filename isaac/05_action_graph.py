"""Tutorial 05 - Your first Action Graph: drive TurtleBot3 Lime with the keyboard (no ROS).

    source scripts/isaac_env.sh
    python isaac/05_action_graph.py          # click into the viewport, then W/S = forward/back, A/D = turn

The graph "KeyboardDrive":

    On Playback Tick ------------------------------------------------------------+
    Read Keyboard State (W) -> To Double -+                                       |
    Read Keyboard State (S) -> To Double -+-> Subtract -> Multiply (x 0.2 m/s) --+-> Differential Controller
    Read Keyboard State (A) -> To Double -+                                       |   (linear / angular velocity)
    Read Keyboard State (D) -> To Double -+-> Subtract -> Multiply (x 0.8 rad/s) -+          |
                                                                                    Articulation Controller (wheels)
A key that is held down reads 1.0, otherwise 0.0, so (W - S) is +1 / 0 / -1.
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from common import lime
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux

stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
GroundPlane("/World/GroundPlane")
for i, (x, y) in enumerate([(1.5, 0.0), (0.0, 1.5), (-1.5, 0.0), (0.0, -1.5)]):
    Cube(f"/World/Landmark_{i}", sizes=0.2, positions=[x, y, 0.1], colors=[0.6, 0.85, 0.2])
articulation_root = lime.add_lime("/World/turtlebot3_lime")

MAX_SPEED, MAX_TURN = 0.2, 0.8  # m/s, rad/s
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/KeyboardDrive", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            *[(f"Key{k}", "omni.graph.nodes.ReadKeyboardState") for k in "WSAD"],
            *[(f"ToDouble{k}", "omni.graph.nodes.ToDouble") for k in "WSAD"],
            ("Forward", "omni.graph.nodes.Subtract"),  # W - S
            ("Turn", "omni.graph.nodes.Subtract"),  # A - D (left is positive)
            ("SpeedScale", "omni.graph.nodes.ConstantDouble"),
            ("TurnScale", "omni.graph.nodes.ConstantDouble"),
            ("Speed", "omni.graph.nodes.Multiply"),
            ("TurnRate", "omni.graph.nodes.Multiply"),
            ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        keys.SET_VALUES: [
            *[(f"Key{k}.inputs:key", k) for k in "WSAD"],
            ("SpeedScale.inputs:value", MAX_SPEED),
            ("TurnScale.inputs:value", MAX_TURN),
            ("DiffController.inputs:wheelRadius", lime.WHEEL_RADIUS),
            ("DiffController.inputs:wheelDistance", lime.WHEEL_BASE),
            ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(articulation_root)]),
            ("ArticulationController.inputs:jointNames", lime.WHEEL_JOINTS),
        ],
        keys.CONNECT: [
            *[(f"Key{k}.outputs:isPressed", f"ToDouble{k}.inputs:value") for k in "WSAD"],
            ("ToDoubleW.outputs:converted", "Forward.inputs:a"),
            ("ToDoubleS.outputs:converted", "Forward.inputs:b"),
            ("ToDoubleA.outputs:converted", "Turn.inputs:a"),
            ("ToDoubleD.outputs:converted", "Turn.inputs:b"),
            ("Forward.outputs:difference", "Speed.inputs:a"),
            ("SpeedScale.inputs:value", "Speed.inputs:b"),
            ("Turn.outputs:difference", "TurnRate.inputs:a"),
            ("TurnScale.inputs:value", "TurnRate.inputs:b"),
            ("OnPlaybackTick.outputs:tick", "DiffController.inputs:execIn"),
            ("Speed.outputs:product", "DiffController.inputs:linearVelocity"),
            ("TurnRate.outputs:product", "DiffController.inputs:angularVelocity"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        ],
    },
)

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(2.2, -2.2, 1.6), target=(0.3, 0.2, 0.1), every_n=3, fps=20)

SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(articulation_root)
print("[05] ready: click into the viewport and use W/S/A/D", flush=True)


# --- --test / --record: "press" keys from the script, the way a user would --------------------------------------
def press(key_name: str, down: bool):
    import carb.input
    import omni.appwindow

    keyboard = omni.appwindow.get_default_app_window().get_keyboard()
    event = carb.input.KeyboardEventType.KEY_PRESS if down else carb.input.KeyboardEventType.KEY_RELEASE
    carb.input.acquire_input_provider().buffer_keyboard_key_event(keyboard, event, getattr(carb.input.KeyboardInput, key_name), 0)


script = [(1.0, "W", True), (4.0, "W", False), (4.5, "A", True), (6.5, "A", False)]  # 3 s forward, 2 s turn left
start = robot.get_world_poses()[0].numpy()[0, :2].copy()
log, steps = {}, 0
while simulation_app.is_running():
    t = steps / 60.0
    if args.test or args.record:
        while script and t >= script[0][0]:
            _, k, down = script.pop(0)
            press(k, down)
        if steps == int(4.4 * 60):
            log["forward"] = float(np.linalg.norm(robot.get_world_poses()[0].numpy()[0, :2] - start))
        if steps == int(6.9 * 60):
            w_, x_, y_, z_ = robot.get_world_poses()[1].numpy()[0]
            log["yaw"] = float(np.degrees(np.arctan2(2 * (w_ * z_ + x_ * y_), 1 - 2 * (y_ * y_ + z_ * z_))))
        if t > 7.5:
            break
    simulation_app.update()
    steps += 1
    if recorder:
        recorder.capture()

ok = True
if args.test:
    checks = {
        f"W held for 3 s: drove {log['forward']:.2f} m (expected ~0.6)": abs(log["forward"] - 0.6) < 0.1,
        f"A held for 2 s: turned left to {log['yaw']:.0f} deg (expected ~90)": 60 < log["yaw"] < 100,
    }
    for text, passed in checks.items():
        print(f"[05][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[05][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
