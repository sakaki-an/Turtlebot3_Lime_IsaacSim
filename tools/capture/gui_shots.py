"""Capture the GUI screenshots used in the tutorials (docs/assets/img/gui/*.png).

Not part of the tutorials - this is how the screenshots on the website are made. It starts the
full Isaac Sim app with a window, drives menus/dialogs like Isaac Sim's own UI tests do
(omni.kit.ui_test), and grabs the whole application window.

    source scripts/isaac_env.sh
    DISPLAY=:10 python tools/capture/gui_shots.py              # all shots
    DISPLAY=:10 python tools/capture/gui_shots.py 02_import     # shots whose name starts with this
"""

import asyncio
import os
import sys
import traceback
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "isaac"))
OUT = REPO / "docs" / "assets" / "img" / "gui"
OUT.mkdir(parents=True, exist_ok=True)
ONLY = sys.argv[1:]

import isaacsim
from isaacsim import SimulationApp

KIT = os.path.join(os.path.dirname(isaacsim.__file__), "apps", "isaacsim.exp.full.kit")
W, H = 1600, 900
app = SimulationApp({"headless": False, "width": W, "height": H, "window_width": W, "window_height": H}, experience=KIT)

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.renderer_capture as rc
import omni.kit.ui_test as ui_test
import omni.ui as ui
import omni.usd
from common import lime
from common.paths import LIME_IMPORTED_USD, LIME_URDF, LIME_USD, USD_DIR
from isaacsim.core.experimental.objects import GroundPlane
from isaacsim.core.rendering_manager import ViewportManager
from pxr import UsdLux

carb.settings.get_settings().set("/app/runLoops/main/rateLimitEnabled", True)
results = {}


async def frames(n=30):
    for _ in range(n):
        await omni.kit.app.get_app().next_update_async()


async def find(query, tries=60, parent=None):
    for _ in range(tries):
        w = (parent or ui_test).find(query) if parent else ui_test.find(query)
        if w is not None:
            return w
        await frames(5)
    raise RuntimeError(f"widget not found: {query}")


async def mouse_away():
    """Park the mouse in the menu bar's empty right part so no tooltip / hover highlight shows."""
    await ui_test.emulate_mouse_move(ui_test.Vec2(W // 2, 10))
    await frames(10)


async def shot(name, crop=None):
    """Capture the whole app window. crop=(left, top, right, bottom) in window pixels (optional).
    The bottom status bar (transient warnings) is always cut off."""
    path = OUT / f"{name}.png"
    await mouse_away()
    await frames(20)
    cap = rc.acquire_renderer_capture_interface()
    cap.capture_next_frame_swapchain(str(path))
    await frames(5)
    cap.wait_async_capture()
    if path.exists():
        from PIL import Image

        img = Image.open(path)
        box = crop or (0, 0, img.width, img.height - 36)
        img.crop(tuple(int(v) for v in box)).save(path)
    results[name] = path.exists()
    print(f"[gui] {'saved' if path.exists() else 'FAILED'} {path.relative_to(REPO)}", flush=True)


def property_sections():
    return [w.widget.title for w in ui_test.find_all("Property//Frame/**/CollapsableFrame[*]") if w.widget.title]


async def scroll_property_to(title_part):
    """Scroll the Property panel so the section whose title contains `title_part` is at the top."""
    print(f"[gui] property sections: {property_sections()}", flush=True)
    target = None
    for w in ui_test.find_all("Property//Frame/**/CollapsableFrame[*]"):
        if w.widget.title and w.widget.title.strip().lower() == title_part.lower():
            target = w
            break
    if target is None:
        raise RuntimeError(f"no Property section containing '{title_part}'")
    scrollers = ui_test.find_all("Property//Frame/**/ScrollingFrame[*]")
    sf = scrollers[0].widget
    for _ in range(3):  # layout may change while scrolling (lazy-built sections)
        delta = target.widget.screen_position_y - sf.screen_position_y - 4
        sf.scroll_y = max(0.0, sf.scroll_y + delta)
        await frames(15)


def select(*paths):
    omni.usd.get_context().get_selection().set_selected_prim_paths(list(paths), True)


async def new_scene(robot_usd=None, variant=True, eye=(1.1, -1.1, 0.75), target=(0.0, 0.0, 0.2)):
    stage_utils.create_new_stage(template="sunlight")
    await frames(10)
    stage = stage_utils.get_current_stage()
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").GetIntensityAttr().Set(500.0)
    GroundPlane("/World/GroundPlane")
    if robot_usd:
        variants = [("Physics", "physx")] if variant else None
        stage_utils.add_reference_to_stage(str(robot_usd), "/World/turtlebot3_lime", variants=variants)
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=eye, target=target)
    await frames(60)
    return stage


# --------------------------------------------------------------------------------------------------
async def print_node_ui_names():
    import omni.graph.core as og

    app_utils.enable_extension("isaacsim.ros2.bridge")
    await frames(10)
    for t in [
        "omni.graph.action.OnPlaybackTick", "omni.graph.action.OnKeyboardInput", "omni.graph.nodes.BreakVector3",
        "isaacsim.core.nodes.IsaacReadSimulationTime", "isaacsim.core.nodes.IsaacArticulationController",
        "isaacsim.core.nodes.IsaacComputeOdometry", "isaacsim.core.nodes.IsaacComputeTransformTree",
        "isaacsim.core.nodes.IsaacCreateRenderProduct", "isaacsim.sensors.physics.IsaacReadJointState",
        "isaacsim.sensors.physics.IsaacReadIMU", "isaacsim.robot.wheeled_robots.DifferentialController",
        "isaacsim.ros2.bridge.ROS2Context", "isaacsim.ros2.bridge.ROS2PublishClock", "isaacsim.ros2.bridge.ROS2SubscribeTwist",
        "isaacsim.ros2.bridge.ROS2PublishOdometry", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
        "isaacsim.ros2.bridge.ROS2PublishTransformTree", "isaacsim.ros2.bridge.ROS2PublishJointState",
        "isaacsim.ros2.bridge.ROS2SubscribeJointState", "isaacsim.ros2.bridge.ROS2CameraHelper",
        "isaacsim.ros2.bridge.ROS2CameraInfoHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        "isaacsim.ros2.bridge.ROS2PublishImu",
    ]:
        nt = og.get_node_type(t)
        name = nt.get_metadata("uiName") if nt else None
        print(f"[gui][uiName] {t} -> {name}", flush=True)


async def shot_02_import_options():
    await new_scene()
    await ui_test.menu_click("File/Import")
    await frames(60)
    dir_field = await find("Select File//Frame/**/StringField[*].identifier=='filepicker_directory_path'")
    await dir_field.input(str(LIME_URDF.parent) + "/")
    await frames(60)
    grid = await find("Select File//Frame/**/VGrid[*].identifier=='filebrowser_grid_view'")
    for _ in range(40):
        f = grid.find(f"**/Label[*].text=='{LIME_URDF.name}'")
        if f is not None:
            break
        await frames(5)
    await f.click()
    await frames(60)
    out_field = await find("Select File//Frame/**/StringField[*].identifier=='urdf_output_path'")
    out_field.model.set_value(str(USD_DIR))
    await frames(20)
    win = ui.Workspace.get_window("Select File")
    try:
        # Crop away the folder tree on the left (it shows unrelated folders of this PC):
        # keep everything from the file grid to the right edge of the dialog.
        left = grid.widget.screen_position_x - 4
        await shot("02_import_options", crop=(left, win.position_y, win.position_x + win.width, win.position_y + win.height))
    finally:
        ui.Workspace.show_window("Select File", False)
        await frames(10)


async def shot_02_physics_variant():
    await new_scene(LIME_IMPORTED_USD, variant=True)
    select("/World/turtlebot3_lime")
    await frames(60)
    await scroll_property_to("Variants")
    await shot("02_physics_variant")


async def shot_03_drive_properties():
    await new_scene(LIME_USD, variant=False)
    select("/World/turtlebot3_lime/Physics/joint2")
    await frames(60)
    await scroll_property_to("Drive")
    await shot("03_drive_properties")


async def shot_03_gain_tuner():
    await new_scene(LIME_USD, variant=False)
    select(f"/World/turtlebot3_lime/{lime.ARTICULATION_ROOT}")
    await frames(20)
    await ui_test.menu_click("Tools/Robotics/Asset Editors/Gain Tuner")
    await frames(120)
    await shot("03_gain_tuner")
    ui.Workspace.show_window("Gain Tuner", False)
    await frames(10)


async def shot_06_action_graph():
    import omni.graph.core as og
    import usdrt.Sdf

    app_utils.enable_extension("isaacsim.ros2.bridge")
    await new_scene(LIME_USD, variant=False)
    root = f"/World/turtlebot3_lime/{lime.ARTICULATION_ROOT}"
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/ROS2_Drive", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinear", "omni.graph.nodes.BreakVector3"),
                ("BreakAngular", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("SubscribeTwist.inputs:topicName", "/cmd_vel"),
                ("DiffController.inputs:wheelRadius", lime.WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", lime.WHEEL_BASE),
                ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(root)]),
                ("ArticulationController.inputs:jointNames", lime.WHEEL_JOINTS),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinear.inputs:tuple"),
                ("BreakLinear.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngular.inputs:tuple"),
                ("BreakAngular.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
            ],
        },
    )
    await frames(10)
    from omni.graph.window.action.action_graph_extension import ActionGraphExtension

    ActionGraphExtension.show_graph([stage_utils.get_current_stage().GetPrimAtPath("/World/ROS2_Drive")])
    await frames(120)
    win = ui.Workspace.get_window("Action Graph")
    win.undock()
    await frames(5)
    win.position_x, win.position_y, win.width, win.height = 40, 60, 1500, 760
    await frames(60)
    # 'F' = frame all nodes (the graph canvas must have focus: move the mouse over it)
    await ui_test.emulate_mouse_move(ui_test.Vec2(800, 450))
    await frames(5)
    await ui_test.emulate_keyboard_press(carb.input.KeyboardInput.F)
    await frames(60)
    await shot("06_action_graph", crop=(40, 60, 1540, 820))


async def dump_labels():
    """Print menu entries and Property sections that the GUI instructions refer to."""
    import omni.kit.menu.utils as mu

    def walk(items, prefix, depth=0):
        if depth > 6 or not items:
            return
        if isinstance(items, dict):
            items = items.get("items", [])
        for it in items:
            name = getattr(it, "name", None)
            if not name:
                continue
            path = f"{prefix}/{name}"
            if any(k in path for k in ("Physics", "Light", "Visual Scripting", "Graph", "Sensor", "Lidar", "LiDAR", "Imu", "IMU", "Camera", "Shape")):
                print(f"[gui][menu] {path}", flush=True)
            walk(getattr(it, "sub_menu", None), path, depth + 1)

    for menu_name, items in mu.get_merged_menus().items():
        if menu_name in ("Create", "Window"):
            walk(items, menu_name)
    await new_scene(LIME_USD, variant=False)
    base = f"/World/turtlebot3_lime/{lime.BASE_LINK}"
    for prim in [base, f"{base}/caster_back_left_link/box", "/World/turtlebot3_lime/PhysicsMaterials/caster"]:
        select(prim)
        await frames(40)
        print(f"[gui][sections] {prim}: {property_sections()}", flush=True)


async def shot_01_overview():
    await new_scene(LIME_USD, variant=False, eye=(1.3, -1.3, 0.9), target=(0.0, 0.0, 0.2))
    select("/World/turtlebot3_lime")
    await frames(60)
    await shot("01_overview")


async def shot_05_keyboard_graph():
    import omni.graph.core as og
    import usdrt.Sdf

    await new_scene(LIME_USD, variant=False)
    root = f"/World/turtlebot3_lime/{lime.ARTICULATION_ROOT}"
    keys = og.Controller.Keys
    og.Controller.edit(  # same graph as isaac/05_action_graph.py
        {"graph_path": "/World/KeyboardDrive", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                *[(f"Key{k}", "omni.graph.nodes.ReadKeyboardState") for k in "WSAD"],
                *[(f"ToDouble{k}", "omni.graph.nodes.ToDouble") for k in "WSAD"],
                ("Forward", "omni.graph.nodes.Subtract"),
                ("Turn", "omni.graph.nodes.Subtract"),
                ("SpeedScale", "omni.graph.nodes.ConstantDouble"),
                ("TurnScale", "omni.graph.nodes.ConstantDouble"),
                ("Speed", "omni.graph.nodes.Multiply"),
                ("TurnRate", "omni.graph.nodes.Multiply"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                *[(f"Key{k}.inputs:key", k) for k in "WSAD"],
                ("SpeedScale.inputs:value", 0.2),
                ("TurnScale.inputs:value", 0.8),
                ("DiffController.inputs:wheelRadius", lime.WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", lime.WHEEL_BASE),
                ("ArticulationController.inputs:targetPrim", [usdrt.Sdf.Path(root)]),
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
    await frames(10)
    from omni.graph.window.action.action_graph_extension import ActionGraphExtension

    ActionGraphExtension.show_graph([stage_utils.get_current_stage().GetPrimAtPath("/World/KeyboardDrive")])
    await frames(120)
    win = ui.Workspace.get_window("Action Graph")
    win.undock()
    await frames(5)
    win.position_x, win.position_y, win.width, win.height = 40, 60, 1500, 760
    await frames(60)
    await ui_test.emulate_mouse_move(ui_test.Vec2(800, 450))
    await frames(5)
    await ui_test.emulate_keyboard_press(carb.input.KeyboardInput.F)
    await frames(60)
    await shot("05_keyboard_graph", crop=(40, 60, 1540, 820))


async def graph_shot(graph_path, name):
    """Open an Action Graph in a big floating editor window, frame all nodes, capture it."""
    from omni.graph.window.action.action_graph_extension import ActionGraphExtension

    ActionGraphExtension.show_graph([stage_utils.get_current_stage().GetPrimAtPath(graph_path)])
    await frames(120)
    win = ui.Workspace.get_window("Action Graph")
    win.undock()
    await frames(5)
    win.position_x, win.position_y, win.width, win.height = 40, 60, 1500, 760
    await frames(60)
    await ui_test.emulate_mouse_move(ui_test.Vec2(800, 450))
    await frames(5)
    await ui_test.emulate_keyboard_press(carb.input.KeyboardInput.F)
    await frames(60)
    await shot(name, crop=(40, 60, 1540, 820))
    win.visible = False
    await frames(10)


async def lime_room_scene(with_robot=True):
    from common.scene import build_lime_room

    stage_utils.create_new_stage(template="empty")
    await frames(10)
    build_lime_room(stage_utils.get_current_stage())
    while stage_utils.is_stage_loading():
        await frames(5)
    if with_robot:
        lime.add_lime("/World/turtlebot3_lime")
    await frames(30)


async def shot_07_odom_graph():
    from common import graphs

    app_utils.enable_extension("isaacsim.ros2.bridge")
    await new_scene(LIME_USD, variant=False)
    root = f"/World/turtlebot3_lime/{lime.ARTICULATION_ROOT}"
    graphs.create_odom_tf_joint_states_graph(root)
    await frames(10)
    await graph_shot("/World/ROS2_OdomTF", "07_odom_tf_graph")


async def shot_12_joint_command_graph():
    from common import graphs

    app_utils.enable_extension("isaacsim.ros2.bridge")
    await new_scene(LIME_USD, variant=False)
    graphs.create_joint_command_graph(f"/World/turtlebot3_lime/{lime.ARTICULATION_ROOT}")
    await frames(10)
    await graph_shot("/World/ROS2_JointCommands", "12_joint_command_graph")


async def shot_08_camera_view():
    from common import sensors
    from omni.kit.viewport.utility import get_active_viewport

    await lime_room_scene()
    cam = sensors.add_d435_camera("/World/turtlebot3_lime")
    await frames(10)
    get_active_viewport().camera_path = cam
    select(cam)
    await frames(120)
    await shot("08_camera_view")
    get_active_viewport().camera_path = "/OmniverseKit_Persp"
    await frames(10)


async def shot_08_camera_graph_tool():
    await lime_room_scene()
    await ui_test.menu_click("Tools/Robotics/ROS 2 OmniGraphs/Camera")
    await frames(90)
    try:
        await shot("08_camera_graph_tool")
    finally:
        ui.Workspace.show_window("ROS2 Camera Graph", False)
        await frames(10)


async def shot_09_lidar_viewport():
    from common import sensors
    from isaacsim.core.simulation_manager import SimulationManager
    await lime_room_scene()
    from common import graphs

    app_utils.enable_extension("isaacsim.ros2.bridge")
    lidar_path = sensors.add_lds_lidar("/World/turtlebot3_lime")
    graphs.create_lidar_graph(lidar_path, show_debug_view=True)
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=(-1.4, -2.4, 3.2), target=(0.9, 0.0, 0.0))
    SimulationManager.setup_simulation(dt=1.0 / 60.0)
    app_utils.play()
    await frames(240)
    await shot("09_lidar_viewport")
    app_utils.stop()
    await frames(10)


async def shot_11_occupancy_map():
    from pxr import Usd, UsdGeom, UsdPhysics

    await lime_room_scene(with_robot=False)
    stage = stage_utils.get_current_stage()
    for name in ("Banana", "Mug", "Bowl", "Bottle"):  # tutorial 11 GUI step: colliders for the objects
        select(f"/World/LimeRoom/Objects/{name}")
        await frames(30)
        await property_add("Physics/Colliders Preset")
        meshes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(f"/World/LimeRoom/Objects/{name}")) if p.IsA(UsdGeom.Mesh)]
        print(f"[gui] {name}: colliders {[p.HasAPI(UsdPhysics.CollisionAPI) for p in meshes]}", flush=True)
    select()
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=(1.0, -0.01, 9.0), target=(1.0, 0.0, 0.0))
    await ui_test.menu_click("Tools/Robotics/Occupancy Map")
    await frames(90)
    fields = ui_test.find_all("Occupancy Map//Frame/**/FloatDrag[*]") or ui_test.find_all("Occupancy Map//Frame/**/FloatField[*]")
    print(f"[gui] occupancy map fields: {len(fields)}", flush=True)
    # widget order in the window: cell size, origin xyz, upper bound xyz, lower bound xyz (values of tutorial 11):
    # origin at 16 cm, the band from 1 cm (-0.15) to 50 cm (+0.34) above the floor
    values = [0.05, 0.5, 0.0, 0.16, 3.7, 2.7, 0.34, -2.7, -2.7, -0.15]
    for w, v in zip(fields, values):
        w.model.set_value(v)
        await frames(2)
    await frames(20)
    buttons = {b.widget.text.strip().lower(): b for b in ui_test.find_all("Occupancy Map//Frame/**/Button[*]") if b.widget.text}
    print(f"[gui] occupancy map buttons: {sorted(buttons)}", flush=True)
    await buttons["calculate"].click()
    await frames(240)
    await buttons["visualize image"].click()
    await frames(120)
    await shot("11_occupancy_map")
    ui.Workspace.show_window("Occupancy Map Visualization", False)
    await frames(10)


async def property_add(menu_path):
    """Click the Property panel's "+ Add" button and pick `menu_path` from its menu (e.g. "Physics/Mass")."""
    buttons = [w for w in ui_test.find_all("Property//Frame/**/Button[*]") if "Add" in (w.widget.text or "")]
    if not buttons:
        raise RuntimeError("no '+ Add' button in the Property panel")
    await buttons[0].click()
    await frames(20)
    await ui_test.select_context_menu(menu_path)
    await frames(30)


async def shot_13_pick_physics():
    """Tutorial 13 GUI steps: physics for the banana and the bowl through the Property panel's + Add menu."""
    from pxr import Usd, UsdGeom, UsdPhysics

    await lime_room_scene(with_robot=False)
    stage = stage_utils.get_current_stage()
    banana, bowl = "/World/LimeRoom/Objects/Banana", "/World/LimeRoom/Objects/Bowl"
    ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=(1.35, -0.75, 0.5), target=(0.85, 0.0, 0.03))
    select(banana)
    await frames(60)
    await property_add("Physics/Rigid Body with Colliders Preset")
    await property_add("Physics/Mass")
    meshes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(banana)) if p.IsA(UsdGeom.Mesh)]
    approx = [UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() for p in meshes]
    print(f"[gui] banana: rigid body {stage.GetPrimAtPath(banana).HasAPI(UsdPhysics.RigidBodyAPI)}, "
          f"mass API {stage.GetPrimAtPath(banana).HasAPI(UsdPhysics.MassAPI)}, mesh colliders {approx}", flush=True)
    select(str(meshes[0].GetPath()))
    await frames(60)
    await scroll_property_to("Collider")
    await shot("13_banana_collider")
    select(bowl)
    await frames(60)
    await property_add("Physics/Colliders Preset")
    bowl_meshes = [p for p in Usd.PrimRange(stage.GetPrimAtPath(bowl)) if p.IsA(UsdGeom.Mesh)]
    print(f"[gui] bowl: rigid body {stage.GetPrimAtPath(bowl).HasAPI(UsdPhysics.RigidBodyAPI)}, mesh colliders "
          f"{[UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get() for p in bowl_meshes]}", flush=True)
    before = {str(p.GetPath()) for p in stage.Traverse()}
    await ui_test.menu_click("Create/Physics/Physics Material")
    await frames(60)
    new = [str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()) not in before]
    print(f"[gui] Create > Physics > Physics Material: new prims {new[:5]}", flush=True)
    await shot("13_physics_material")
    rigid = [w for w in ui_test.find_all("**/CheckBox[*]")]
    print(f"[gui] dialog checkboxes: {len(rigid)}", flush=True)
    labels = [w for w in ui_test.find_all("**/Label[*]") if (w.widget.text or "").strip() == "Rigid Body Material"]
    if labels:
        await labels[0].click()
        await frames(10)
    ok = [w for w in ui_test.find_all("**/Button[*]") if (w.widget.text or "").strip() == "Ok"]
    if ok:
        await ok[0].click()
        await frames(60)
    new = [str(p.GetPath()) for p in stage.Traverse() if str(p.GetPath()) not in before]
    print(f"[gui] after Ok: new prims {new[:5]}", flush=True)
    lime.add_lime("/World/turtlebot3_lime")
    await frames(60)
    finger = next(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == "gripper_left_link")
    for path in (banana, finger, str(meshes[0].GetPath())):
        select(path)
        await frames(60)
        print(f"[gui] sections for {path.split('/')[-1]}: {property_sections()}", flush=True)


SHOTS = [
    ("00_labels", dump_labels),
    ("00_node_names", print_node_ui_names),
    ("01_overview", shot_01_overview),
    ("02_import_options", shot_02_import_options),
    ("02_physics_variant", shot_02_physics_variant),
    ("03_drive_properties", shot_03_drive_properties),
    ("03_gain_tuner", shot_03_gain_tuner),
    ("05_keyboard_graph", shot_05_keyboard_graph),
    ("06_action_graph", shot_06_action_graph),
    ("07_odom_graph", shot_07_odom_graph),
    ("08_camera_view", shot_08_camera_view),
    ("08_camera_graph_tool", shot_08_camera_graph_tool),
    ("09_lidar_viewport", shot_09_lidar_viewport),
    ("11_occupancy_map", shot_11_occupancy_map),
    ("12_joint_command_graph", shot_12_joint_command_graph),
    ("13_pick_physics", shot_13_pick_physics),
]


async def main():
    await frames(200)  # let the UI settle
    for name, fn in SHOTS:
        if ONLY and not any(name.startswith(o) for o in ONLY):
            continue
        try:
            print(f"[gui] --- {name}", flush=True)
            await fn()
        except Exception:
            results[name] = False
            print(f"[gui] ERROR in {name}:\n{traceback.format_exc()}", flush=True)


task = asyncio.ensure_future(main())
while not task.done():
    app.update()
print(f"[gui] results: {results}", flush=True)
app.close()
