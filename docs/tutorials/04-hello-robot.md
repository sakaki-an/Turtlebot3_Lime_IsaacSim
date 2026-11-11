# 04 · Move the robot from Python ("Hello Robot")

<p class="tutorial-meta">⏱ 20 min · Level: beginner · Script: <code>isaac/04_hello_robot.py</code></p>

**Goal.** Control the Lime's wheels, arm and gripper directly from a Python script, without ROS and without graphs.
This is the fastest way to try things out, and how reinforcement-learning and test scripts talk to robots.

<figure class="result" markdown>
<video controls autoplay loop muted playsinline>
  <source src="../../assets/video/tutorial04_hello_robot.mp4" type="video/mp4">
</video>
<figcaption>Forward 0.6 m, turn left 90°, reach out with the arm, open and close the gripper, fold the arm.</figcaption>
</figure>

## Concepts

**Articulation.** A robot in Isaac Sim is an *articulation*: a tree of rigid links connected by joints, simulated
together. `Articulation(path_of_the_root)` gives you access to all its **DoF** (degrees of freedom, the movable
joints) at once:

```python
robot = Articulation("/World/turtlebot3_lime/Geometry/base_footprint/base_link")
robot.dof_names    # ['wheel_left_joint', 'wheel_right_joint', 'joint1', ..., 'gripper_right_joint']
wheels = robot.get_dof_indices(["wheel_left_joint", "wheel_right_joint"])
```

| You want | Call | Uses the drive from tutorial 03 |
|---|---|---|
| wheel speed | `set_dof_velocity_targets([[wl, wr]], dof_indices=wheels)` | velocity drive (damping) |
| arm / gripper position | `set_dof_position_targets([[...]], dof_indices=arm)` (move it gradually, see below) | position drive (stiffness + damping) |
| read the state | `get_dof_positions()`, `get_dof_velocities()`, `get_world_poses()` | |

Angles are in **radians**, prismatic joints (the gripper) in **metres**.

**Differential drive in Python.** `DifferentialController` from `isaacsim.robot.experimental.wheeled_robots`
computes the same formula as the graph node in tutorial 06:

```python
controller = DifferentialController(wheel_radius=0.06, wheel_base=0.29425)
wheel_speeds = controller.forward(np.array([0.2, 0.0]))   # 0.2 m/s forward -> [3.33, 3.33] rad/s
robot.set_dof_velocity_targets([wheel_speeds], dof_indices=wheels)
```

## Run it

```bash
source scripts/isaac_env.sh
python isaac/04_hello_robot.py
python isaac/04_hello_robot.py --headless --test
```

The script runs a small timed program:

```python
program = [
    (0.5, lambda: drive(0.2, 0.0)),                          # forward 0.2 m/s for 3 s -> 0.6 m
    (3.5, lambda: drive(0.0, math.pi / 2 / 2.0)),            # turn left: 90 deg in 2 s
    (5.5, lambda: drive(0.0, 0.0)),
    (6.0, lambda: move_arm(REACH)),                          # smoothly, in 1.5 s (see below)
    (8.0, lambda: robot.set_dof_position_targets([[0.019]], dof_indices=gripper)),   # open
    (9.0, lambda: robot.set_dof_position_targets([[-0.010]], dof_indices=gripper)),  # close
    (10.0, lambda: move_arm(lime.ARM_HOME)),
]
```

```text
[04][check] ok   drove 0.58 m forward (commanded 0.6 m)
[04][check] ok   turned left to 84.2 deg (commanded 90)
[04][check] ok   arm reached the reach pose (max error 0.002 rad)
[04][check] ok   gripper opened to 0.0190 m
[04][check] ok   base stayed flat on the floor while the arm moved (max tilt 0.0 deg)
[04][test] PASS
```

The small differences come from acceleration: the robot needs a moment to reach the commanded speed.

### Move the arm smoothly, never in one jump

You *could* send the arm straight to its target: `robot.set_dof_position_targets([REACH], dof_indices=arm)`. The
position drives then swing the arm at the servos' full speed (about 4.7 rad/s) with an instant start and stop, and
the reaction tips the whole robot. The Lime's wheels sit near the front, and its centre of mass is only about 4 cm
behind the wheel axle, so the rear lifts off the floor. We measured:

| How the arm moves | Base tilts up to |
|---|---|
| target in one jump (full servo speed) | 19.6° reaching out, **46.7°** folding back |
| the same, with the battery mass doubled to 3 kg | 1.4° reaching out, 16.7° folding back |
| speed limited to 1 rad/s | 0.5° |
| **smoothly in 1.5 s** (`move_arm`) | **0.04°** |

A real arm controller never jumps either: MoveIt (tutorial 12) plans trajectories with speed and acceleration
limits. `move_arm` does the simplest version: it moves the target a little every frame, with a smooth start and
stop.

```python
--8<-- "isaac/04_hello_robot.py:move_arm"
```

!!! note "Only the left gripper finger is commanded"
    `gripper_right_joint` follows `gripper_left_joint` by itself, through its mimic joint (tutorial 03).

??? example "Full script: `isaac/04_hello_robot.py`"
    ```python
    --8<-- "isaac/04_hello_robot.py"
    ```

**Next:** [05 · Your first Action Graph](05-action-graph.md)
