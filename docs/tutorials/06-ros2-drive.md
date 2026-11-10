# 06 · Drive TurtleBot3 Lime with ROS 2 (`/cmd_vel`)

<p class="tutorial-meta">⏱ 20 min · Level: beginner · Script: <code>isaac/06_ros2_drive.py</code></p>

**Goal.** Build an **Action Graph** that publishes the simulation clock and turns ROS 2 `geometry_msgs/Twist`
messages on `/cmd_vel` into wheel speeds. Then drive the Lime with your keyboard from the ROS 2 container.

<figure class="result" markdown>
<video controls autoplay loop muted playsinline>
  <source src="../../assets/video/tutorial06_ros2_drive.mp4" type="video/mp4">
</video>
<figcaption><code>ros2 topic pub /cmd_vel … {linear: {x: 0.25}, angular: {z: 0.5}}</code> from the ROS 2 container: the Lime drives a circle.</figcaption>
</figure>

## Concepts

**OmniGraph / Action Graph.** Isaac Sim connects simulation and ROS 2 with *graphs* of nodes.
An **Action Graph** runs when an *event* fires. `On Playback Tick` fires once per simulated frame while the
simulation is playing. ROS 2 topics are only published or received **while you are in Play**.

**ROS 2 bridge.** The extension `isaacsim.ros2.bridge` provides the ROS 2 nodes (`ROS2 Publish Clock`,
`ROS2 Subscribe Twist`, …). It runs on Isaac Sim's bundled Jazzy libraries (see [Setup](00-setup.md)).

**Differential drive.** A Twist gives the robot's forward speed $v$ and turn rate $\omega$.
The `Differential Controller` node converts them to wheel speeds:

$$
\omega_{left} = \frac{v - \omega\,\frac{L}{2}}{r}, \qquad \omega_{right} = \frac{v + \omega\,\frac{L}{2}}{r}
$$

For the Lime the wheel radius is $r = 0.06$ m and the wheel separation is $L = 0.29425$ m (from the URDF).

## The graph

```mermaid
flowchart LR
  tick([On Playback Tick])
  time[Isaac Read<br/>Simulation Time]
  clock[ROS2 Publish Clock<br/><i>/clock</i>]
  twist[ROS2 Subscribe Twist<br/><i>/cmd_vel</i>]
  bl[Break Vector3]
  ba[Break Vector3]
  diff[Differential Controller<br/>r = 0.06, L = 0.29425]
  art[Articulation Controller<br/>wheel_left_joint, wheel_right_joint]
  tick --> clock
  time -- simulationTime --> clock
  tick --> twist
  tick --> art
  twist -- execOut --> diff
  twist -- linearVelocity --> bl -- x --> diff
  twist -- angularVelocity --> ba -- z --> diff
  diff -- velocityCommand --> art
```

## Build it

=== "GUI"

    1. Open a stage with the tuned robot: **File ▸ Open** `assets/usd/turtlebot3_lime_tuned.usda` and add a
       ground plane. Or add it as a reference to your own scene.
    2. **Window ▸ Graph Editors ▸ Action Graph**, then **New Action Graph**.
    3. Search for each node by name in the node library (left side) and drag it into the graph:
       `On Playback Tick`, `Isaac Read Simulation Time`, `ROS2 Publish Clock`, `ROS2 Subscribe Twist`,
       2× `Break 3-Vector`, `Differential Controller`, `Articulation Controller`.
    4. Connect them as in the diagram above.
    5. Set the inputs in the **Property** panel:

        | Node | Input | Value |
        |---|---|---|
        | ROS2 Subscribe Twist | topicName | `/cmd_vel` |
        | Differential Controller | wheelRadius / wheelDistance / maxWheelSpeed | `0.06` / `0.29425` / `8.063` |
        | Articulation Controller | targetPrim | `…/Geometry/base_footprint/base_link` (the articulation root) |
        | Articulation Controller | jointNames | `wheel_left_joint`, `wheel_right_joint` |

    6. Press **Play** ▶.

    !!! tip "Shortcuts"
        Isaac Sim has graph builders for common cases. **Tools ▸ Robotics ▸ OmniGraph Controllers ▸ Differential Controller**
        creates the controller half of this graph. **Tools ▸ Robotics ▸ ROS 2 OmniGraphs** creates ROS 2 graphs
        for clock, TF, odometry, cameras and LiDAR. Building this one by hand once is the best way to understand what those shortcuts produce.

    <div class="shot-todo">📸 Screenshot pending: <code>gui/06_action_graph.png</code> (the graph above in the Action Graph editor).</div>

=== "Python (standalone)"

    ```bash
    source scripts/isaac_env.sh
    python isaac/06_ros2_drive.py
    ```

    `og.Controller.edit()` creates nodes, sets values and connects ports in one call. These are the
    same three steps you do by hand in the GUI:

    ```python
    og.Controller.edit(
        {"graph_path": "/World/ROS2_Drive", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                # ...
            ],
            keys.SET_VALUES: [("DiffController.inputs:wheelRadius", 0.06), ...],
            keys.CONNECT: [("DiffController.outputs:velocityCommand",
                            "ArticulationController.inputs:velocityCommand"), ...],
        },
    )
    ```

    Two more details in the script:

    - `app_utils.enable_extension("isaacsim.ros2.bridge")`: standalone apps start without the bridge.
    - The main loop is capped at 60 Hz (`/app/runLoops/main/rateLimitEnabled`). A headless script
      otherwise runs faster than real time, and ROS tools such as teleop, RViz and Nav2 expect
      roughly real time.

    ??? example "Full script: `isaac/06_ros2_drive.py`"
        ```python
        --8<-- "isaac/06_ros2_drive.py"
        ```

## Drive it from ROS 2

With the simulation playing, open a second terminal:

=== "Keyboard"

    ```bash
    ./docker/run_ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ```

    `i` forward, `,` backward, `j` / `l` turn, `k` stop. `q` / `z` change the speed.

=== "One command"

    ```bash
    ./docker/run_ros.sh ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 0.25}, angular: {z: 0.5}}"
    ```

Look at the clock as well:

```bash
./docker/run_ros.sh ros2 topic hz /clock        # ≈ 60 Hz
```

!!! success "Automatic check"
    `python isaac/06_ros2_drive.py --headless --test` waits for `/cmd_vel` messages, then checks that the
    robot moved more than 0.3 m. With the `topic pub` command above it reports
    `robot moved 0.75 m … PASS`.

![Driving](../assets/img/tutorial06_ros2_drive.png){ .full }

## Troubleshooting

| Symptom | Fix |
|---|---|
| `/cmd_vel` is published but the robot does not move | Is the simulation **playing**? Is `targetPrim` the *articulation root* (`…/base_link`)? |
| The robot turns the wrong way or drives backwards | Check the `jointNames` order: left first, then right. |
| The robot is slow or jittery | You are not using the tuned asset (see [03](03-tune-robot.md)). |

**Next:** odometry, TF and RViz, coming soon.
