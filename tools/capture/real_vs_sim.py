"""The real TurtleBot3 Lime next to the simulated one - pictures for the website (not part of the tutorials).

    source scripts/isaac_env.sh
    python tools/capture/real_vs_sim.py PHOTO.jpg

Writes docs/assets/img/real_lime.jpg (the photo, resized, metadata removed) and docs/assets/img/real_vs_sim.jpg
(photo | Isaac Sim render with the tuned robot, arm posed and camera placed roughly like in the photo).
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "isaac"))
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common import lime
from common.paths import IMG_DIR
from common.recorder import Recorder
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from PIL import Image
from pxr import UsdLux

PHOTO = Path(sys.argv[1])
POSE = np.array([-0.25, -0.9, 1.75, 0.0, 0.7, 0.0])  # upper arm leaning back, forearm reaching forward (as in the photo)

stage = stage_utils.create_new_stage(template="sunlight")
UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(700.0)
GroundPlane("/World/GroundPlane")
root = lime.add_lime("/World/turtlebot3_lime")
recorder = Recorder("real_vs_sim", eye=(0.60, 0.52, 0.60), target=(0.0, 0.0, 0.19), resolution=(1200, 900), focal_length=24.0)
SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
simulation_app.update()
robot = Articulation(root)
arm = robot.get_dof_indices(lime.ARM_JOINTS)
start = robot.get_dof_positions(dof_indices=arm).numpy()[0]
for i in range(1, 181):  # move smoothly (a jump would jolt the robot)
    a = 0.5 - 0.5 * np.cos(np.pi * i / 180)
    robot.set_dof_position_targets([list(start + a * (POSE - start))], dof_indices=arm)
    simulation_app.update()
for _ in range(60):
    simulation_app.update()
recorder.capture(force=True)
sim = Image.fromarray(recorder.frames[-1])
app_utils.stop()

photo = Image.open(PHOTO).convert("RGB")  # convert() drops the metadata (EXIF)
photo.thumbnail((1600, 1600))
photo.save(IMG_DIR / "real_lime.jpg", quality=85)
h = 600
left = photo.resize((int(photo.width * h / photo.height), h))
right = sim.resize((int(sim.width * h / sim.height), h))
both = Image.new("RGB", (left.width + right.width + 8, h), (255, 255, 255))
both.paste(left, (0, 0))
both.paste(right, (left.width + 8, 0))
both.save(IMG_DIR / "real_vs_sim.jpg", quality=88)
print(f"[real_vs_sim] saved {IMG_DIR / 'real_lime.jpg'} and {IMG_DIR / 'real_vs_sim.jpg'}", flush=True)
simulation_app.close()
