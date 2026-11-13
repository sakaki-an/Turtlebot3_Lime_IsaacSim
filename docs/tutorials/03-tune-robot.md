# 03 · Tune the robot

<p class="tutorial-meta">⏱ 25 min · Level: beginner → intermediate · Scripts: <code>isaac/03_tune_robot.py</code>, <code>isaac/03_before_after.py</code></p>

**Goal.** Make the imported Lime simulate well. The arm should hold its pose, the wheels should follow
velocity commands, the casters should slide, and the gripper should open and close. Then make it **look like the
real robot**. The tuning goes into its own USD file, so a re-import never loses it.

<figure class="result" markdown>
<video controls autoplay loop muted playsinline>
  <source src="../../assets/video/tutorial03_before_after.mp4" type="video/mp4">
</video>
<figcaption>Before and after, the same test for both: wait 2 s, both wheels at 5 rad/s for 3 s, open the gripper.
<b>Left</b>, the robot as tutorial 02 imported it (given only a wheel drive, so that its wheels can turn at all): the
arm falls over, the robot covers about 0.25 m and turns about 18°, and the gripper stays shut.
<b>Right</b>, tuned: the arm holds the folded "home" pose, the robot drives 0.87 m straight (ideal 0.90 m), and the
gripper opens.</figcaption>
</figure>

## Concepts

### Joint drives

Every movable joint has a **drive**, a spring-damper that PhysX solves *implicitly*, so it stays stable even when stiff:

$$
\tau = k_p\,(q_{target} - q) + k_d\,(\dot q_{target} - \dot q), \qquad |\tau| \le \tau_{max}
$$

| Use | stiffness $k_p$ | damping $k_d$ | target |
|---|---|---|---|
| **Position control** (arm, gripper) | > 0 | > 0 | position |
| **Velocity control** (wheels) | **0** | > 0 | velocity |

$\tau_{max}$ is **Max Force**. The importer took it from the URDF `effort` limits (wheels 3 N·m, arm 1 N·m)
but left stiffness and damping at 0, which is why everything was limp in tutorial 02.

!!! warning "Units: USD stores angular gains *per degree*"
    In the Property panel and in the `.usda` file, revolute stiffness is **N·m/deg** and damping is
    **N·m·s/deg**. Isaac Sim's Python APIs (`Articulation.set_dof_gains`) use **radians**.
    200 N·m/rad = 3.49 N·m/deg. The script converts with `gain * π / 180`.

### Friction and *combine modes*

Where two colliders touch, PhysX **combines** their friction coefficients. The default is the **average**.
Our casters are meant to slide, so they get friction 0. The floor (a ground plane with no physics
material of its own) uses PhysX's default material, which has friction > 0. The average of the two is
**not** 0. The caster pads carry the heavy battery at the back, and that friction brakes the robot.
Setting the caster material's combine mode to **min** makes its 0.0 win.

Measured on the final tuned asset (both wheels at 5 rad/s for 3 s, ideal distance 0.90 m):

| Caster material | Floor | Distance | Wheel speed |
|---|---|---|---|
| friction 0, combine *average* | default material | 0.67 m | 1.2 / 2.4 rad/s (and it curves) |
| friction 0, combine *average* | friction set to 0 | 0.87 m | 5.0 rad/s |
| friction 0, combine **min** | default material | **0.87 m** | **5.0 rad/s** |

The second row confirms the cause: removing friction at the caster–floor contact fixes the problem.
The third row fixes it without touching the floor.

The robot on the left of the video at the top drives even less, about 0.25 m. Its caster pads have no physics
material at all, so they keep PhysX's default friction instead of 0. Give that robot only the caster material
(friction 0, combine *min*), with its arm still fallen over and its cylinder tyres, and it drives 0.86 m in the
same test.

### Wheel colliders: cylinder vs sphere

The URDF describes each tyre as a cylinder, 0.06 m radius and 24 mm wide. A cylinder touches the floor along a
**line** across its tread. When the robot turns, the inner and outer end of that line would need different
speeds, so the tyre scrubs and resists the turn. A **sphere** with the same radius touches the floor in one
**point**, which is the ideal wheel that the differential-drive formula (tutorial 06) assumes.

Measured: command 0.2 m/s and 0.5 rad/s, then read the robot's real yaw rate:

| Tyre collider | Wheel speeds (command 2.11 / 4.56 rad/s) | Yaw rate (command 0.5 rad/s) |
|---|---|---|
| cylinder (URDF) | 2.40 / 4.16 rad/s | 0.44 rad/s |
| **sphere** | **2.06 / 4.56 rad/s** | **0.50 rad/s** |

So the tuned asset switches the cylinder's collision off and adds an invisible sphere collider (`sphere_collider`,
purpose *guide*) to each wheel link. The visible tyre mesh is unchanged.

### Looks: the real robot's colours

The URDF paints the whole robot grey. The real Lime is black and lime green: black waffle plates and Dynamixel
servos, lime-green 3D-printed brackets, translucent yellow-green tyres with lime hubs, metal standoffs, and the
LiDAR on a lime mount. The colours below were measured from a photo of the real robot.

![The real TurtleBot3 Lime (left, photo: Momoi Open Source) and the tuned simulated Lime (right)](../assets/img/real_vs_sim.jpg){ .full }

One mesh per link is too coarse: `link2.stl`, for example, holds a servo, a bracket and four screws. But inside the
mesh these are separate **solids**: groups of triangles that don't share a vertex with the rest. The script splits
every mesh into its solids and colours each one by its size: a 28 × 42 × 46 mm block is a Dynamixel servo
(black), other arm parts are brackets (lime), the wheel's 120 mm ring is the tyre and its 84 mm disc the hub.
Only the look changes. The colliders, the masses and the joints are untouched.

??? example "The colours and the rules: `isaac/common/looks.py`"
    ```python
    --8<-- "isaac/common/looks.py:colours"
    ```

    ```python
    --8<-- "isaac/common/looks.py:classify"
    ```

!!! info "Instanced meshes"
    The importer stores each link's visual mesh as an *instance*: a shared, read-only copy. To colour parts of
    it, the script first switches instancing off for the visual meshes (`SetInstanceable(False)`, an override in
    the tuned layer). The colliders stay instanced.

### Solver iterations

An articulation (the robot's joint tree) is solved iteratively. Raising the articulation's
**position iterations from 32 (default) to 64** made the drive test slightly more accurate:
0.847 m → 0.874 m in the run above. Keep **velocity iterations ≤ 4**. PhysX's TGS solver changed how it
treats more, and it prints a warning.

## The values

| What | Setting |
|---|---|
| Wheels (×2) | velocity drive: stiffness 0, damping 100 N·m·s/rad, max force 3 N·m (XM430-W210) |
| Arm joints (×6) | position drive: stiffness 200 N·m/rad, damping 10 N·m·s/rad, max force 4.1 N·m (XM430-W350 stall torque) |
| Arm start pose | the SRDF `home` pose `[0, -1.10, 1.80, 0, 0.98, 0]` rad, as both the drive target and the initial joint state (upstream uses 1.87 for `joint3`, which is outside its own URDF limit of 1.8567; MoveIt rejects that in tutorial 12. 1.80 leaves a margin for the arm sagging under gravity) |
| Gripper (left) | position drive: stiffness 1000 N/m, damping 20 N·s/m, max force 10 N |
| Gripper (right) | no drive. It follows the left finger through its mimic joint |
| Tyres / casters | friction 1.0 (average) / friction 0.0 (**min**) |
| Tyre collision shape | **sphere**, r = 0.06 m, instead of the URDF cylinder |
| Articulation | 64 position / 4 velocity solver iterations |
| Look | black plates and servos, lime brackets, translucent yellow-green tyres, lime hubs, metal standoffs (`common/looks.py`) |

## Tune it

=== "GUI"

    Start from the stage of tutorial 02 (robot referenced, `Physics = physx`, ground plane).

    **Drives.** In the Stage panel open `turtlebot3_lime ▸ Physics` and select a joint, for example `joint2`.
    In **Property ▸ Drive ▸ Angular** set *Stiffness*, *Damping* and *Max Force* (in **per-degree** units, see the table above).
    Set *Target Position* in **degrees**. Select several joints at once (Ctrl+click) to edit them together.

    ![Property ▸ Drive ▸ Angular of joint2 (tuned values, per-degree units)](../assets/img/gui/03_drive_properties.png){ .full }

    **Gain Tuner.** **Tools ▸ Robotics ▸ Asset Editors ▸ Gain Tuner** shows all joints of the selected
    articulation in one table. It can also play test motions (sinusoids or steps) and plot how each
    joint follows them. Use it to see the effect of a gain before you commit to it.

    ![Gain Tuner](../assets/img/gui/03_gain_tuner.png){ .full }

    **Friction.** **Create ▸ Physics ▸ Physics Material** (rigid body). In its **Rigid Body Material** section
    set *Static/Dynamic Friction*. In its **Material (PhysX)** section set the *Friction Combine Mode* to `min`
    for the caster material. Then select each caster collider (`caster_back_*_link/box`) and choose the
    material in **Property ▸ Physics materials on selected models**. Do not use the *Materials on
    selected models* section: that one is for the visual look.

    **Sphere tyres** (these GUI steps describe what the script does; the script is the verified path).
    For each `wheel_*_link`: select its `cylinder` and untick **Collider ▸ Collision Enabled**.
    Then right-click the link and choose **Create ▸ Shape ▸ Sphere**. Set its *Radius* to `0.06` and *Purpose* to
    `guide` (not rendered). Add a collider with **Add ▸ Physics ▸ Collider** and bind the tyre physics material.

    **Solver.** Select `base_link` (the articulation root). In its **Articulation Root** / **PhysxArticulationAPI**
    properties set *Solver Position Iteration Count* = 64 and *Solver Velocity Iteration Count* = 4.

    **Looks** (these GUI steps describe what the script does; the script is the verified path). **Create ▸
    Material ▸ OmniPBR** for each colour and set its *Albedo* colour. Select a link's visual prim, for example
    `link4/link4`, untick **Instanceable** in the Property panel (instanced prims are read-only), then pick the
    material under **Materials on selected models**. In the GUI you colour whole meshes; colouring the solids
    *inside* a mesh separately (servo black, bracket lime) is what the script adds.

    **Save as an override layer.** Instead of editing the imported files, create a new stage and add the
    imported robot as a reference (**File ▸ Add Reference**). Make your edits there, then **File ▸ Save As**
    `assets/usd/turtlebot3_lime_tuned.usda`. Your edits are stored as *overs* on top of the
    reference. That is exactly what the script below produces.

=== "Python (standalone)"

    ```bash
    source scripts/isaac_env.sh
    python isaac/03_tune_robot.py --headless --test
    ```

    The script writes `assets/usd/turtlebot3_lime_tuned.usda`, a small layer that *references* the
    import and adds overrides. Then it loads that file and checks the result.

    ```python
    robot = stage.DefinePrim("/turtlebot3_lime", "Xform")
    robot.GetReferences().AddReference("./turtlebot3_lime/turtlebot3_lime.usda")
    robot.GetVariantSet("Physics").SetVariantSelection("physx")      # (1)!

    drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
    drive.CreateStiffnessAttr(200.0 * math.pi / 180.0)                # (2)!
    drive.CreateDampingAttr(10.0 * math.pi / 180.0)
    drive.CreateMaxForceAttr(4.1)
    drive.CreateTargetPositionAttr(math.degrees(home))                 # (3)!

    PhysxSchema.PhysxMaterialAPI.Apply(caster_material).CreateFrictionCombineModeAttr("min")

    looks.paint_like_the_real_lime(stage, "/turtlebot3_lime")       # (4)!
    ```

    1. Users of the tuned file no longer have to pick the physics variant.
    2. N·m/rad → N·m/deg.
    3. USD joint angles are in degrees too.
    4. The real robot's colours, solid by solid (see *Looks* above).

    ??? example "Full script: `isaac/03_tune_robot.py`"
        ```python
        --8<-- "isaac/03_tune_robot.py"
        ```

## Check the result

```text
[03] painted like the real Lime: black 645338 faces, servo 38830 faces, lime 45098 faces, tyre 5848 faces, hub 8444 faces, metal 3864 faces
[03] arm max error after 2 s : 0.002 rad
[03] distance in 3 s         : 0.873 m (ideal 0.90 m)
[03] gripper left / right    : 0.0190 / 0.0190 m (target 0.019)
[03][test] PASS
```

- The arm holds `home` against gravity to within 0.002 rad (0.1°).
- 5 rad/s × 0.06 m = 0.30 m/s, so 0.90 m in 3 s. The missing 2.6 cm is the acceleration phase.
- The right finger copies the left one exactly: the mimic joint works under PhysX.

<figure class="result" markdown>
<video controls autoplay loop muted playsinline>
  <source src="../../assets/video/tutorial03_tuned.mp4" type="video/mp4">
</video>
<figcaption><code>03_tune_robot.py</code> with the tuned robot: the arm holds, the robot drives, the gripper opens.</figcaption>
</figure>

!!! tip "Experiment"
    Change `ARM_DRIVE = dict(stiffness=200.0, ...)` to `stiffness=20.0` and re-run: the arm sags.
    Change the caster combine mode back to `"average"`: the robot covers only about 0.67 m, curves, and the
    distance test fails.

### Before and after, side by side

`isaac/03_before_after.py` makes the video at the top of this page. It puts the imported robot next to the tuned
one and runs the same test on both. It only reads the two assets, so run `03_tune_robot.py` first.

```bash
python isaac/03_before_after.py                     # with a window
python isaac/03_before_after.py --headless --test
```

The left robot is the asset exactly as tutorial 02 imported it:

```python
--8<-- "isaac/03_before_after.py:robots"
```

with one change. Its wheel drives have no damping, so a speed command would do nothing at all. The script gives
them the tuned robot's damping, in the running simulation only:

```python
--8<-- "isaac/03_before_after.py:one_change"
```

```text
[03vs] as imported : arm moved 1.819 rad in 2 s, drove 0.251 m (sideways +0.044 m, turned +17.5 deg), wheels 3.06 / 1.26 rad/s, gripper 1.1 mm
[03vs] tuned       : arm moved 0.001 rad in 2 s, drove 0.874 m (sideways +0.001 m, turned +0.1 deg), wheels 5.00 / 5.01 rad/s, gripper 19.0 mm
[03vs][check] ok   tuned: arm holds its pose (moved 0.001 rad)
[03vs][check] ok   tuned: drove 0.87 m (ideal 0.90 m)
[03vs][check] ok   tuned: gripper open (19.0 mm)
[03vs][check] ok   as imported: arm falls (moved 1.82 rad)
[03vs][check] ok   as imported: gripper does not open (1.1 mm)
[03vs][test] PASS
```

- The imported arm has no stiffness, so it falls over by about 105° in the first 2 s.
- The imported wheels turn at only about 3 and 1 rad/s instead of 5, and the robot covers a quarter of a metre
  and veers. The caster pads cause it (see [Friction and combine modes](#friction-and-combine-modes)).
- The imported gripper has no drive, so the command to open it does nothing.
- The left robot's drive is sensitive to small changes in the scene: moving the robots by a few centimetres gave
  0.23–0.25 m and 17–19°. So the check tests only its arm and gripper. The tuned robot must pass the same test
  as above.

??? example "Full script: `isaac/03_before_after.py`"
    ```python
    --8<-- "isaac/03_before_after.py"
    ```

From now on every tutorial loads **`assets/usd/turtlebot3_lime_tuned.usda`**. In Python that is
`common.lime.add_lime()`.

**Next:** [04 · Move it from Python](04-hello-robot.md)
