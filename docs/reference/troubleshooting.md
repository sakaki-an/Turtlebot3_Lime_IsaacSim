# Troubleshooting

Problems we actually hit while building these tutorials on Isaac Sim 6.2 (pip install, Ubuntu 24.04),
and how to fix them.

## Isaac Sim

### Core extensions fail to load (`charset_normalizer`)

**Symptom.** The GUI (`isaacsim`) starts, but the console shows

```text
Failed to import python module isaacsim.core.experimental.prims ... charset_normalizer.md.MessDetectorPlugin
size changed, may indicate binary incompatibility. Expected 24 from C header, got 16 from PyObject
[ext: isaacsim.core.experimental.prims-1.12.1] Failed to startup python extension.
```

and many robotics features are missing.

**Cause.** Several Kit extensions in a pip install bundle their own, differently compiled copies of
the `charset_normalizer` package. When the full GUI app starts, an import chain
(`scipy` → `numpy.f2py` → `charset_normalizer`) mixes modules from two copies.

**Fix.** `scripts/setup_isaacsim.sh` adds a one-line `.pth` file to the venv that imports
`charset_normalizer` once at interpreter start, so every later import reuses one consistent copy:

```bash
echo "import charset_normalizer, charset_normalizer.md, charset_normalizer.cd" \
  > .venv/lib/python3.12/site-packages/zz_preload_charset_normalizer.pth
```

### `ROS2 Bridge startup failed … libament_index_cpp.so: cannot open shared object file`

The bundled ROS 2 libraries are not on `LD_LIBRARY_PATH`. Run `source scripts/isaac_env.sh` in the
terminal **before** starting Isaac Sim. The dynamic linker reads `LD_LIBRARY_PATH` only at process start,
so setting it later inside Isaac Sim does not help.

### `pip install` with `uv` fails to resolve `isaacsim`

`uv pip install isaacsim…` stops with `no version of mujoco-usd-converter==…` or
`tinyobjloader==2.0.0rc13 … pre-releases weren't enabled`. Use plain `pip` inside the venv, as NVIDIA's
guide and `scripts/setup_isaacsim.sh` do.

### The robot has no physics / no articulation is found

An asset made by the 6.2 URDF importer has a `Physics` variant set with **no default selection**.
Select `physx` (Property ▸ Variants), or pass `variants=[("Physics", "physx")]` to
`add_reference_to_stage`. The tuned asset `turtlebot3_lime_tuned.usda` already selects it.

### `Undefined 'xformOp:translate' property … Set the 'reset_xform_op_properties' parameter to True`

The experimental `XformPrim` API wants a standard transform stack before it moves a prim:
`XformPrim(path, reset_xform_op_properties=True).set_world_poses(...)`.

### The robot drives slowly or curves although both wheels get the same command

The casters drag on the floor. Use the tuned asset and see [03 · Tune the robot](../tutorials/03-tune-robot.md):
set the caster material's friction combine mode to `min`.

## ROS 2

### Topics are listed but `ros2 topic echo` shows nothing

That is a DDS transport problem. Both sides must use the same `ROS_DOMAIN_ID` and RMW, and here the
Fast DDS UDP-only profile in `config/fastdds.xml`. The container uses `--network host`. Check also that
the simulation is **playing**: the ROS 2 bridge publishes only during Play.

### Building the Docker image is very slow

`apt` mirrors may be slow from your network. Use a closer mirror:
`UBUNTU_MIRROR=http://<mirror>/ubuntu/ ./docker/run_ros.sh`. It replaces both `archive.ubuntu.com`
and `security.ubuntu.com`.

### RViz does not open from the container

`run_ros.sh` passes your `DISPLAY` and runs `xhost +local:docker`. Start it from a desktop terminal,
or set `DISPLAY` (for example `DISPLAY=:0` or your VNC display) before calling it.
