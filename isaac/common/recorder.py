"""Record what the simulation looks like - this is how the tutorial videos were made.

Usage (after SimulationApp is created and the stage is built):

    rec = Recorder("tutorial_02_import", eye=(1.2, 1.2, 0.8), target=(0, 0, 0.2))
    while running:
        simulation_app.update()
        rec.capture()            # grabs a frame (every ``every_n`` calls)
    rec.save()                   # -> docs/assets/video/<name>.mp4 + docs/assets/img/<name>.png

It uses a dedicated USD camera + a Replicator RGB annotator, so it works with and without a window.
"""

from __future__ import annotations

import numpy as np

from .paths import IMG_DIR, MEDIA_DIR


class Recorder:
    def __init__(
        self,
        name: str,
        eye=(1.5, 1.5, 1.0),
        target=(0.0, 0.0, 0.2),
        resolution=(1280, 720),
        focal_length: float = 18.0,
        every_n: int = 2,
        fps: int = 30,
        camera_path: str = "/World/RecorderCamera",
    ):
        import omni.replicator.core as rep
        from pxr import UsdGeom

        import isaacsim.core.experimental.utils.stage as stage_utils

        self.name = name
        self.every_n = max(1, every_n)
        self.fps = fps
        self.frames: list[np.ndarray] = []
        self._calls = 0
        stage = stage_utils.get_current_stage()
        cam = UsdGeom.Camera.Define(stage, camera_path)
        cam.CreateFocalLengthAttr().Set(focal_length)
        cam.CreateClippingRangeAttr().Set((0.01, 1000.0))
        self._xform_op = UsdGeom.Xformable(cam.GetPrim()).AddTransformOp()
        self.look_at(eye, target)
        self._render_product = rep.create.render_product(camera_path, resolution)
        self._annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self._annotator.attach([self._render_product])

    def look_at(self, eye, target):
        """Move the recording camera (call it every frame for a follow-cam)."""
        from pxr import Gf

        view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*map(float, eye)), Gf.Vec3d(*map(float, target)), Gf.Vec3d(0, 0, 1))
        self._xform_op.Set(view.GetInverse())

    def capture(self, force: bool = False) -> None:
        self._calls += 1
        if not force and (self._calls % self.every_n) != 0:
            return
        data = self._annotator.get_data()
        if data is None or getattr(data, "size", 0) == 0:
            import omni.replicator.core as rep
            import omni.timeline

            playing = omni.timeline.get_timeline_interface().is_playing()
            if playing and not force:
                return  # the renderer is still warming up; later calls get frames
            # Render one frame explicitly (e.g. a still of a stage that is not playing). Never pause the
            # timeline: rep.orchestrator.step() does that by default, and the simulation would stop.
            rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
            data = self._annotator.get_data()
        if data is not None and getattr(data, "size", 0) > 0:
            self.frames.append(np.asarray(data)[:, :, :3].copy())

    def save(self, still_index: int = -1) -> tuple[str | None, str | None]:
        """Write <name>.mp4 (all frames) and <name>.png (one still, default: last frame)."""
        import imageio.v2 as imageio

        if not self.frames:
            print(f"[recorder] no frames captured for '{self.name}'")
            return None, None
        MEDIA_DIR.mkdir(parents=True, exist_ok=True)
        IMG_DIR.mkdir(parents=True, exist_ok=True)
        mp4 = MEDIA_DIR / f"{self.name}.mp4"
        png = IMG_DIR / f"{self.name}.png"
        imageio.imwrite(png, self.frames[still_index])
        if len(self.frames) > 1:
            imageio.mimwrite(mp4, self.frames, fps=self.fps, codec="libx264", quality=6, macro_block_size=8)
        print(f"[recorder] saved {png} and {mp4} ({len(self.frames)} frames)")
        return str(mp4), str(png)

    def detach(self) -> None:
        self._annotator.detach()
        self._render_product.destroy()
