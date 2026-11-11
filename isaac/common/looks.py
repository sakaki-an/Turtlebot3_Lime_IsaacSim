"""Make the simulated TurtleBot3 Lime look like the real one (tutorial 03).

The URDF paints the whole robot grey. The real Lime has black waffle plates and Dynamixel servos, lime-green
3D-printed brackets, translucent yellow-green tyres with lime hubs, metal standoffs, and a black LiDAR on a lime
mount (colours measured from a photo of the real robot, see docs/assets/img/real_lime.jpg).

Each link's visual mesh comes from one STL file, but inside it are several separate solids: a servo, a bracket,
a few screws. So we split every mesh into its solids (connected triangles) and colour each solid by what it is,
judged by its size. Only the look changes: colliders, masses and joints are untouched.
"""

from __future__ import annotations

import numpy as np

# --8<-- [start:colours]
# name: (base colour as linear RGB, roughness, metallic, opacity) - UsdPreviewSurface values
COLOURS = {
    "black": ((0.005, 0.005, 0.005), 0.55, 0.0, 1.0),  # waffle plates, LiDAR, gripper
    "servo": ((0.008, 0.008, 0.008), 0.30, 0.0, 1.0),  # Dynamixel XM430 servos (slightly glossy)
    "lime": ((0.16, 0.62, 0.012), 0.45, 0.0, 1.0),  # 3D-printed PLA brackets
    "tyre": ((0.55, 0.65, 0.03), 0.20, 0.0, 0.85),  # translucent yellow-green tyres
    "hub": ((0.30, 0.62, 0.03), 0.40, 0.0, 1.0),  # lime wheel hubs
    "metal": ((0.60, 0.60, 0.62), 0.30, 1.0, 1.0),  # aluminium standoffs, screws
}
# --8<-- [end:colours]


def solids(mesh) -> list[tuple[np.ndarray, np.ndarray]]:
    """Split a triangle mesh into its solids (triangles connected through shared vertices).
    Returns [(face indices, sorted size in mm), ...]."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    points = np.asarray(mesh.GetPointsAttr().Get())
    counts = np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    indices = np.asarray(mesh.GetFaceVertexIndicesAttr().Get())
    first = np.concatenate([[0], np.cumsum(counts)[:-1]])
    face_of_corner = np.repeat(np.arange(len(counts)), counts)
    # connect each corner with the first corner of its face -> every face is one connected piece
    graph = coo_matrix((np.ones(len(indices)), (indices[first[face_of_corner]], indices)), shape=(len(points),) * 2)
    n, label = connected_components(graph, directed=False)
    lo = np.full((n, 3), np.inf)
    hi = np.full((n, 3), -np.inf)
    np.minimum.at(lo, label, points)
    np.maximum.at(hi, label, points)
    extent = hi - lo
    scale = 1.0 if np.nanmax(extent[np.isfinite(extent)]) > 5.0 else 1000.0  # metres or millimetres
    face_label = label[indices[first]]
    order = np.argsort(face_label, kind="stable")
    cuts = np.flatnonzero(np.diff(face_label[order])) + 1
    return [(faces, np.sort(extent[face_label[faces[0]]] * scale)) for faces in np.split(order, cuts)]


# --8<-- [start:classify]
def classify(mesh_name: str, size: np.ndarray, faces: int) -> str:
    """Which colour is this solid? size = its sorted extents in mm, faces = its number of triangles."""
    small, middle, large = size
    if mesh_name == "tire":
        return "tyre" if large > 100 else "hub"  # the outer ring vs the inner disc
    if mesh_name == "ladar":
        return "lime" if small < 15 and large > 50 else "black"  # the LiDAR sits on a lime mount plate
    if mesh_name == "base":
        if small < 8 and middle < 8 and large > 15:
            return "metal"  # standoffs: thin rods
        return "black"  # plates, battery, boards
    if mesh_name.startswith("gripper") or mesh_name == "link7":
        return "black"  # the gripper and its servo
    if faces < 300 or large < 15:
        return "metal"  # screws, small parts
    if 26 <= small <= 31 and 33 <= middle <= 44 and 39 <= large <= 48:
        return "servo"  # Dynamixel XM430: 28.5 x 34 x 46.5 mm
    return "lime"  # brackets
# --8<-- [end:classify]


def paint_like_the_real_lime(stage, root: str = "/turtlebot3_lime") -> dict:
    """Give every visual mesh of the robot under `root` the real robot's colours. Returns {colour: faces}."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    # 1. one material per colour
    materials = {}
    for name, (colour, roughness, metallic, opacity) in COLOURS.items():
        mat = UsdShade.Material.Define(stage, f"{root}/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"{root}/Looks/{name}/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*colour))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        materials[name] = mat

    # 2. the visual meshes are instanced (shared, read-only); make them editable. Colliders stay instanced.
    def is_visual_instance(prim):
        meshes = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
        return meshes and not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in meshes)

    for _level in range(5):  # instances can contain instances: open them level by level
        todo = [p.GetPath() for p in Usd.PrimRange(stage.GetPrimAtPath(root)) if p.IsInstance()
                and is_visual_instance(p) and "d435" not in str(p.GetPath())]  # the camera keeps its own materials
        if not todo:
            break
        for path in todo:
            stage.GetPrimAtPath(path).SetInstanceable(False)

    # 3. colour each solid of each visual mesh
    painted = {name: 0 for name in COLOURS}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(root)):
        if not prim.IsA(UsdGeom.Mesh) or prim.HasAPI(UsdPhysics.CollisionAPI) or "d435" in str(prim.GetPath()):
            continue
        mesh = UsdGeom.Mesh(prim)
        by_colour: dict[str, list] = {}
        for faces, size in solids(mesh):
            colour = classify(prim.GetName(), size, len(faces))
            by_colour.setdefault(colour, []).append(faces)
        main = max(by_colour, key=lambda c: sum(len(f) for f in by_colour[c]))
        binding = UsdShade.MaterialBindingAPI.Apply(prim)
        binding.Bind(materials[main])  # the colour with the most faces: the whole mesh; subsets for the others
        for colour, groups in by_colour.items():
            faces = np.sort(np.concatenate(groups))
            painted[colour] += len(faces)
            if colour == main:
                continue
            subset = UsdGeom.Subset.CreateGeomSubset(
                mesh, colour, UsdGeom.Tokens.face, [int(i) for i in faces], UsdShade.Tokens.materialBind
            )
            UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(materials[colour])
    return painted
