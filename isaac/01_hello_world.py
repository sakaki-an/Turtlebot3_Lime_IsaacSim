"""Tutorial 01 - Isaac Sim basics: your first standalone script ("Hello World").

    source scripts/isaac_env.sh
    python isaac/01_hello_world.py              # with a window
    python isaac/01_hello_world.py --headless --test

Every standalone script follows the same four steps:
  1. start the simulator          SimulationApp(...)        <- must come first
  2. build a scene (a USD stage)  ground, light, objects, robot
  3. press Play                   SimulationManager + app_utils.play()
  4. step the simulation          simulation_app.update() in a loop, read and write the scene
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

# --- 1. Start Isaac Sim. Import NOTHING from isaacsim / omni / pxr before this line. ------------------------
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720})

# Now the Isaac Sim / Omniverse / USD modules can be imported.
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from common import lime
from isaacsim.core.experimental.objects import Cube, GroundPlane
from isaacsim.core.experimental.prims import Articulation, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from pxr import UsdLux, UsdPhysics

# --- 2. Build a scene ----------------------------------------------------------------------------------------
stage = stage_utils.create_new_stage(template="sunlight")  # a new, empty USD stage with a distant light
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)  # soft light from the sky
GroundPlane("/World/GroundPlane")  # a floor with a collider

# A 20 cm cube, 1 m above the floor. A plain Cube only has geometry; the two USD *physics schemas* make it fall
# (RigidBodyAPI: it has mass and moves) and collide (CollisionAPI: it touches other colliders).
cube = Cube("/World/LimeCube", sizes=0.2, positions=[0.6, 0.0, 1.0], colors=["limegreen"])
UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath("/World/LimeCube"))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath("/World/LimeCube"))

# The robot of these tutorials (made in tutorials 02 + 03). It is a USD *reference* to another file.
articulation_root = lime.add_lime("/World/turtlebot3_lime")

recorder = None
if args.record:
    from common.recorder import Recorder

    recorder = Recorder(args.record, eye=(1.4, -1.4, 0.9), target=(0.3, 0.0, 0.25))

# --- 3. Press Play ----------------------------------------------------------------------------------------------
SimulationManager.setup_simulation(dt=1.0 / 60.0)  # physics at 60 steps per second
app_utils.play()

# --- 4. Step and observe --------------------------------------------------------------------------------------------
robot = Articulation(articulation_root)
steps = 0
while simulation_app.is_running():
    simulation_app.update()  # renders one frame and advances physics by dt
    steps += 1
    if recorder:
        recorder.capture()
    if steps % 30 == 0 and steps <= 120:
        z = float(XformPrim("/World/LimeCube").get_world_poses()[0].numpy()[0, 2])
        print(f"[01] t = {steps / 60:.1f} s   cube height = {z:.3f} m", flush=True)
    if (args.test or args.record) and steps >= 150:
        break

ok = True
if args.test:
    cube_z = float(XformPrim("/World/LimeCube").get_world_poses()[0].numpy()[0, 2])
    robot_z = float(robot.get_world_poses()[0].numpy()[0, 2])
    checks = {
        f"cube landed on the floor (centre at {cube_z:.3f} m = half its size)": abs(cube_z - 0.1) < 0.01,
        f"robot stands on its wheels (base_link at {robot_z:.3f} m = wheel radius)": abs(robot_z - 0.06) < 0.01,
        f"robot has {robot.num_dofs} joints that can move": robot.num_dofs == 10,
    }
    for text, passed in checks.items():
        print(f"[01][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    print(f"[01][test] {'PASS' if ok else 'FAIL'}")
if recorder:
    recorder.save()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
