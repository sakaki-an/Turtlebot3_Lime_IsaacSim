"""Tutorial 11 (part 1) - Make a Nav2 map of the lime_room with Isaac Sim's occupancy map generator.

    source scripts/isaac_env.sh
    python isaac/11_make_map.py --headless

Writes ros2_ws/src/lime_isaac_bringup/maps/lime_room.pgm + lime_room.yaml (the ROS map_server format).

The generator (extension isaacsim.asset.gen.omap) checks, cell by cell, whether a physics collider occupies a
height band of the scene, and flattens the band into one 2D map. We map everything from 1 cm to 50 cm above the
floor. A map is for what stays put: the walls, the pillar, the crate and the divider. The floor is cleared first
(no objects); things that come and go, like the traffic cone of 11_nav2.py, are the costmaps' job.
"""

import sys

from common.cli import parse_args

args = parse_args(
    __doc__.splitlines()[0],
    extra=lambda p: p.add_argument("--out", default=None, help="write the map to this folder instead"),
)

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import numpy as np
from common.paths import IMG_DIR, REPO_ROOT
from common.scene import CONE_XY, ROOM_X, ROOM_Y, WALL_THICKNESS, build_lime_room
from isaacsim.core.simulation_manager import SimulationManager

RESOLUTION = 0.05  # m per cell (the usual value for small indoor robots)
BAND = (0.01, 0.50)  # m above the floor: from just above the floor (not the floor itself) to above the obstacles
MAP_DIR = REPO_ROOT / "ros2_ws" / "src" / "lime_isaac_bringup" / "maps"
if args.out:
    MAP_DIR = REPO_ROOT / args.out

app_utils.enable_extension("isaacsim.asset.gen.omap")
simulation_app.update()
import omni.physx
import omni.usd
from isaacsim.asset.gen.omap.bindings import _omap

stage = stage_utils.create_new_stage(template="empty")
build_lime_room(stage, with_objects=False)  # only what stays put - no objects, no robot (not in its own map)
while stage_utils.is_stage_loading():
    simulation_app.update()

# The generator uses PhysX queries: the simulation must be playing.
SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
for _ in range(10):
    simulation_app.update()

# Map the inside of the room plus the walls (bounds are offsets from the origin).
pad = WALL_THICKNESS + 0.1
origin = (0.5, 0.0, 0.16)  # any FREE point inside the room
lo = (ROOM_X[0] - pad - origin[0], ROOM_Y[0] - pad - origin[1], BAND[0] - origin[2])
hi = (ROOM_X[1] + pad - origin[0], ROOM_Y[1] + pad - origin[1], BAND[1] - origin[2])
generator = _omap.Generator(omni.physx.get_physx_interface(), omni.usd.get_context().get_stage_id())
generator.update_settings(RESOLUTION, 4.0, 5.0, 6.0)  # cell size, occupied / free / unknown marker values
generator.set_transform(origin, lo, hi)
generator.generate2d()
occupied = np.array([(p.x, p.y) for p in generator.get_occupied_positions()])
free = np.array([(p.x, p.y) for p in generator.get_free_positions()])
app_utils.stop()
print(f"[11] generator: {len(occupied)} occupied, {len(free)} free cells", flush=True)

# --- Rasterise into a ROS map image: row 0 = largest y (north), column 0 = smallest x (west) ----------------
x_min, y_min = origin[0] + lo[0], origin[1] + lo[1]
width = int(round((hi[0] - lo[0]) / RESOLUTION))
height = int(round((hi[1] - lo[1]) / RESOLUTION))
grid = np.full((height, width), 205, dtype=np.uint8)  # 205 = unknown (grey)


def paint(points, value):
    cols = np.floor((points[:, 0] - x_min) / RESOLUTION).astype(int)
    rows = height - 1 - np.floor((points[:, 1] - y_min) / RESOLUTION).astype(int)
    keep = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    grid[rows[keep], cols[keep]] = value


paint(free, 254)  # free (white)
paint(occupied, 0)  # occupied (black)

MAP_DIR.mkdir(parents=True, exist_ok=True)
from PIL import Image

Image.fromarray(grid, mode="L").save(MAP_DIR / "lime_room.pgm")
(MAP_DIR / "lime_room.yaml").write_text(
    "image: lime_room.pgm\n"
    "mode: trinary\n"
    f"resolution: {RESOLUTION}\n"
    f"origin: [{x_min:.3f}, {y_min:.3f}, 0.0]  # world position of the bottom-left pixel\n"
    "negate: 0\n"
    "occupied_thresh: 0.65\n"
    "free_thresh: 0.25\n"
)
print(f"[11] map {width} x {height} cells ({width * RESOLUTION:.2f} x {height * RESOLUTION:.2f} m) -> {MAP_DIR}", flush=True)

ok = True
if args.test or args.record:
    # Spot checks against the known room: a wall cell must be occupied, the robot's start must be free.
    def cell(x, y):
        return grid[height - 1 - int((y - y_min) / RESOLUTION), int((x - x_min) / RESOLUTION)]

    checks = {
        "north wall (1.0, 2.55) occupied": cell(1.0, ROOM_Y[1] + 0.05) == 0,
        "pillar (2.0, -1.2) occupied": cell(2.0, -1.2) == 0,
        "robot start (0, 0) free": cell(0.0, 0.0) == 254,
        "room centre (1.0, -0.5) free": cell(1.0, -0.5) == 254,
        f"where the cone will stand {CONE_XY} free (not in the map)": cell(*CONE_XY) == 254,
    }
    for text, passed in checks.items():
        print(f"[11][check] {'ok  ' if passed else 'FAIL'} {text}")
    ok = all(checks.values())
    if args.test:
        print(f"[11][test] map {'PASS' if ok else 'FAIL'}")
    if args.record:
        Image.fromarray(grid, mode="L").resize((width * 4, height * 4), Image.NEAREST).save(IMG_DIR / f"{args.record}.png")

simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
