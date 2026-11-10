"""Tutorial 00 - Check the setup: Isaac Sim starts, the ROS 2 bridge loads, /clock reaches ROS 2.

    source scripts/isaac_env.sh
    python isaac/00_check_setup.py --headless

    # while it runs, in another terminal:
    ./docker/run_ros.sh ros2 topic echo /clock --once

The script publishes the simulation time on /clock for 60 seconds (or until you close the window).
If the container prints a clock message, Isaac Sim and ROS 2 can talk to each other.
"""

import sys

from common.cli import parse_args

args = parse_args(__doc__.splitlines()[0])

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": args.headless})

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

# 1. Enable the ROS 2 bridge. If the bundled ROS 2 libraries cannot be loaded, Isaac Sim logs
#    "ROS2 Bridge startup failed" - see the Troubleshooting page.
app_utils.enable_extension("isaacsim.ros2.bridge")
settings = carb.settings.get_settings()
settings.set("/app/runLoops/main/rateLimitEnabled", True)  # run in real time (60 Hz)
settings.set("/app/runLoops/main/rateLimitFrequency", 60)
simulation_app.update()

import omni.graph.core as og

# 2. A minimal Action Graph: every frame, publish the simulation time on /clock.
stage_utils.create_new_stage(template="empty")
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ClockGraph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        ],
    },
)

# 3. Press Play - ROS 2 topics are only published while the simulation is playing.
SimulationManager.setup_simulation(dt=1.0 / 60.0)
app_utils.play()
print("[00] publishing /clock - run:  ./docker/run_ros.sh ros2 topic echo /clock --once", flush=True)

received = []
if args.test:
    # Self-check: listen to /clock with the rclpy that ships with Isaac Sim (loaded by the bridge).
    import rclpy
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node("lime_setup_check")
    node.create_subscription(Clock, "/clock", lambda msg: received.append(msg.clock.sec + msg.clock.nanosec * 1e-9), 10)

frames = 0
while simulation_app.is_running() and frames < 60 * 60:
    simulation_app.update()
    frames += 1
    if args.test:
        rclpy.spin_once(node, timeout_sec=0.0)
        if len(received) > 10 or frames > 600:
            break

ok = True
if args.test:
    ok = len(received) > 10
    last = f"{received[-1]:.2f} s" if received else "-"
    print(f"[00][test] received {len(received)} /clock messages (last: {last}) -> {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()
app_utils.stop()
simulation_app.close(exit_code=0 if ok else 1)
sys.exit(0 if ok else 1)
