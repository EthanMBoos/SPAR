"""Shared Blender mechanics for visible SPAR world generation.

This module is imported inside BlenderMCP ``execute_blender_code`` calls.  It
owns cross-family mechanics; prompts still own layout and appearance. Repeated
environment-family logic belongs in ``worldgen/families/<family>/helpers.py``
rather than expanding this module into one universal scene generator.
"""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
from typing import Sequence

import bpy
from mathutils import Matrix, Vector


def reset_scene() -> None:
    """Remove scene data without changing preferences or disabling add-ons."""
    bpy.ops.object.select_all(action="SELECT")
    bpy.ops.object.delete(use_global=False)
    for collection in list(bpy.data.collections):
        bpy.data.collections.remove(collection)
    for datablocks in (
        bpy.data.meshes,
        bpy.data.curves,
        bpy.data.cameras,
        bpy.data.lights,
        bpy.data.materials,
    ):
        for datablock in list(datablocks):
            if datablock.users == 0:
                datablocks.remove(datablock)


def ensure_collection(name: str) -> bpy.types.Collection:
    collection = bpy.data.collections.get(name)
    if collection is None:
        collection = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(collection)
    return collection


def move_to_collection(obj: bpy.types.Object, collection_name: str) -> None:
    collection = ensure_collection(collection_name)
    for existing in list(obj.users_collection):
        existing.objects.unlink(obj)
    collection.objects.link(obj)


def configure_scene() -> None:
    scene = bpy.context.scene
    engine_items = scene.bl_rna.properties["render"].fixed_type.properties[
        "engine"
    ].enum_items
    engines = {item.identifier for item in engine_items}
    for candidate in ("BLENDER_EEVEE_NEXT", "BLENDER_EEVEE"):
        if candidate in engines:
            scene.render.engine = candidate
            break
    scene.render.resolution_x = 1920
    scene.render.resolution_y = 1080
    scene.render.resolution_percentage = 100
    transforms = {
        item.name for item in scene.view_settings.bl_rna.properties["view_transform"].enum_items
    }
    if "AgX" in transforms:
        scene.view_settings.view_transform = "AgX"


def _as_vector(values: Sequence[float], length: int) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != length:
        raise ValueError(f"expected {length} values, got {len(result)}")
    return result


def dimensions_from_anchor(
    anchor: bpy.types.Object, fallback: Sequence[float] = (1.0, 1.0, 1.0)
) -> tuple[float, float, float]:
    raw = anchor.get("spar_plan_dimensions")
    if raw is None:
        return _as_vector(fallback, 3)
    if isinstance(raw, str):
        raw = ast.literal_eval(raw)
    return _as_vector(raw, 3)


def guide(name: str) -> bpy.types.Object:
    """Return one named plan guide without relying on naming heuristics."""
    obj = bpy.data.objects.get(name)
    if obj is None or not obj.get("spar_plan_kind"):
        raise ValueError(f"missing plan guide: {name}")
    return obj


def guide_frame(
    name: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """Return an unparented plan guide's center and stored full dimensions."""
    obj = guide(name)
    return tuple(float(value) for value in obj.location), dimensions_from_anchor(obj)


def site(name: str) -> bpy.types.Object:
    """Return one named SPAR site."""
    obj = bpy.data.objects.get(name)
    if obj is None or not obj.get("spar_site"):
        raise ValueError(f"missing SPAR site: {name}")
    return obj


def plan_guides(kind: str | None = None) -> list[bpy.types.Object]:
    guides = [obj for obj in bpy.data.objects if obj.get("spar_plan_kind")]
    if kind is not None:
        guides = [obj for obj in guides if obj.get("spar_plan_kind") == kind]
    return sorted(guides, key=lambda obj: obj.name)


def assembly_anchors(*plan_types: str) -> list[bpy.types.Object]:
    wanted = set(plan_types)
    anchors = plan_guides("assembly")
    if wanted:
        anchors = [obj for obj in anchors if obj.get("spar_plan_type") in wanted]
    return anchors


def roots_by_type(*spar_types: str) -> list[bpy.types.Object]:
    wanted = set(spar_types)
    roots = [obj for obj in bpy.data.objects if obj.get("spar_type")]
    if wanted:
        roots = [obj for obj in roots if obj.get("spar_type") in wanted]
    return sorted(roots, key=lambda obj: obj.name)


def roots_by_role(role: str) -> list[bpy.types.Object]:
    return sorted(
        [obj for obj in bpy.data.objects if obj.get("spar_role") == role],
        key=lambda obj: obj.name,
    )


def _require_new_object_name(name: str) -> None:
    if bpy.data.objects.get(name) is not None:
        raise ValueError(f"object already exists: {name}")


def add_empty(
    name: str,
    location: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    display_type: str = "PLAIN_AXES",
    display_size: float = 1.0,
    collection: str = "SPAR_PLAN",
) -> bpy.types.Object:
    _require_new_object_name(name)
    obj = bpy.data.objects.new(name, None)
    ensure_collection(collection).objects.link(obj)
    obj.location = _as_vector(location, 3)
    obj.rotation_euler = _as_vector(rotation, 3)
    obj.empty_display_type = display_type
    obj.empty_display_size = float(display_size)
    return obj


def add_plan_guide(
    name: str,
    kind: str,
    location: Sequence[float],
    dimensions: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    color: Sequence[float] = (0.2, 0.6, 1.0, 0.35),
) -> bpy.types.Object:
    dims = _as_vector(dimensions, 3)
    obj = add_empty(
        name,
        location,
        rotation=rotation,
        display_type="CUBE",
        display_size=1.0,
    )
    obj.scale = tuple(max(value / 2.0, 0.02) for value in dims)
    obj.color = _as_vector(color, 4)
    obj["spar_plan_kind"] = kind
    obj["spar_plan_dimensions"] = list(dims)
    return obj


def add_plan_anchor(
    name: str,
    plan_type: str,
    zone: str,
    location: Sequence[float],
    dimensions: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    role: str | None = None,
) -> bpy.types.Object:
    dims = _as_vector(dimensions, 3)
    if any(value <= 0.0 for value in dims):
        raise ValueError(f"anchor dimensions must be positive: {dims}")
    obj = add_empty(
        name,
        location,
        rotation=rotation,
        display_type="ARROWS",
        display_size=0.8,
    )
    obj["spar_plan_kind"] = "assembly"
    obj["spar_plan_type"] = plan_type
    obj["spar_plan_zone"] = zone
    obj["spar_plan_dimensions"] = list(dims)
    obj["spar_plan_origin"] = "base"
    if role:
        obj["spar_plan_role"] = role
    return obj


def add_rect_boundary_anchors(
    boundary: bpy.types.Object,
    *,
    gate_side: str = "south",
    gate_offset: float = 0.0,
    gate_width: float = 4.0,
    fence_height: float = 1.8,
    gate_height: float = 2.2,
    thickness: float = 0.1,
) -> tuple[list[bpy.types.Object], bpy.types.Object]:
    """Place five positive-size fence anchors and one gate on a rectangle."""
    if gate_side not in {"south", "north", "east", "west"}:
        raise ValueError("gate_side must be south, north, east, or west")
    width, depth, _ = dimensions_from_anchor(boundary)
    side_length = width if gate_side in {"south", "north"} else depth
    before = side_length / 2.0 + float(gate_offset) - float(gate_width) / 2.0
    after = side_length / 2.0 - float(gate_offset) - float(gate_width) / 2.0
    if min(before, after, gate_width, fence_height, gate_height, thickness) <= 0.0:
        raise ValueError("gate and fence dimensions must fit inside the boundary")

    origin = Vector(boundary.location)
    angle = float(boundary.rotation_euler.z)
    axis_x = Vector((math.cos(angle), math.sin(angle), 0.0))
    axis_y = Vector((-math.sin(angle), math.cos(angle), 0.0))

    def world(local_x: float, local_y: float) -> Vector:
        return origin + axis_x * local_x + axis_y * local_y

    specs: list[tuple[str, Vector, tuple[float, float, float], float]] = []
    if gate_side in {"south", "north"}:
        gate_y = -depth / 2.0 if gate_side == "south" else depth / 2.0
        opposite = "North" if gate_side == "south" else "South"
        opposite_y = depth / 2.0 if gate_side == "south" else -depth / 2.0
        before_center = -width / 2.0 + before / 2.0
        after_center = width / 2.0 - after / 2.0
        specs.extend(
            [
                (f"Fence_{opposite}", world(0.0, opposite_y), (width, thickness, fence_height), angle),
                ("Fence_West", world(-width / 2.0, 0.0), (depth, thickness, fence_height), angle + math.pi / 2.0),
                ("Fence_East", world(width / 2.0, 0.0), (depth, thickness, fence_height), angle + math.pi / 2.0),
                (f"Fence_{gate_side.title()}_Before", world(before_center, gate_y), (before, thickness, fence_height), angle),
                (f"Fence_{gate_side.title()}_After", world(after_center, gate_y), (after, thickness, fence_height), angle),
            ]
        )
        gate_location = world(float(gate_offset), gate_y)
        gate_rotation = angle
    else:
        gate_x = -width / 2.0 if gate_side == "west" else width / 2.0
        opposite = "East" if gate_side == "west" else "West"
        opposite_x = width / 2.0 if gate_side == "west" else -width / 2.0
        before_center = -depth / 2.0 + before / 2.0
        after_center = depth / 2.0 - after / 2.0
        specs.extend(
            [
                (f"Fence_{opposite}", world(opposite_x, 0.0), (depth, thickness, fence_height), angle + math.pi / 2.0),
                ("Fence_South", world(0.0, -depth / 2.0), (width, thickness, fence_height), angle),
                ("Fence_North", world(0.0, depth / 2.0), (width, thickness, fence_height), angle),
                (f"Fence_{gate_side.title()}_Before", world(gate_x, before_center), (before, thickness, fence_height), angle + math.pi / 2.0),
                (f"Fence_{gate_side.title()}_After", world(gate_x, after_center), (after, thickness, fence_height), angle + math.pi / 2.0),
            ]
        )
        gate_location = world(gate_x, float(gate_offset))
        gate_rotation = angle + math.pi / 2.0

    fences = [
        add_plan_anchor(
            name, "fence", boundary.name, location, dimensions,
            rotation=(0.0, 0.0, rotation),
        )
        for name, location, dimensions, rotation in specs
    ]
    gate = add_plan_anchor(
        f"Gate_{gate_side.title()}", "gate", boundary.name, gate_location,
        (gate_width, thickness, gate_height),
        rotation=(0.0, 0.0, gate_rotation),
    )
    return fences, gate


def add_plan_anchor_row(
    name_prefix: str,
    plan_type: str,
    zone: str,
    center: Sequence[float],
    dimensions: Sequence[float],
    count: int,
    spacing: float,
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
) -> list[bpy.types.Object]:
    """Create a centered row of distinct assembly anchors along local X."""
    if count < 1:
        raise ValueError("count must be positive")
    origin = Vector(_as_vector(center, 3))
    rot = _as_vector(rotation, 3)
    direction = Vector((math.cos(rot[2]), math.sin(rot[2]), 0.0))
    midpoint = (count - 1) / 2.0
    return [
        add_plan_anchor(
            f"{name_prefix}_{index + 1:02d}",
            plan_type,
            zone,
            origin + direction * ((index - midpoint) * float(spacing)),
            dimensions,
            rotation=rot,
        )
        for index in range(count)
    ]


def add_plan_anchor_grid(
    name_prefix: str,
    plan_type: str,
    zone: str,
    center: Sequence[float],
    dimensions: Sequence[float],
    rows: int,
    columns: int,
    spacing: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
) -> list[bpy.types.Object]:
    """Create a centered local-XY grid of distinct assembly anchors."""
    if rows < 1 or columns < 1:
        raise ValueError("rows and columns must be positive")
    spacing_x, spacing_y = _as_vector(spacing, 2)
    origin = Vector(_as_vector(center, 3))
    rot = _as_vector(rotation, 3)
    cos_z, sin_z = math.cos(rot[2]), math.sin(rot[2])
    axis_x = Vector((cos_z, sin_z, 0.0))
    axis_y = Vector((-sin_z, cos_z, 0.0))
    anchors = []
    for row in range(rows):
        for column in range(columns):
            local_x = (column - (columns - 1) / 2.0) * spacing_x
            local_y = (row - (rows - 1) / 2.0) * spacing_y
            anchors.append(
                add_plan_anchor(
                    f"{name_prefix}_{len(anchors) + 1:02d}",
                    plan_type,
                    zone,
                    origin + axis_x * local_x + axis_y * local_y,
                    dimensions,
                    rotation=rot,
                )
            )
    return anchors


def add_site(
    name: str,
    site: str,
    location: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    order: int | None = None,
    display_size: float = 1.0,
) -> bpy.types.Object:
    obj = add_empty(
        name,
        location,
        rotation=rotation,
        display_type="ARROWS",
        display_size=display_size,
        collection="SPAR_SITES",
    )
    obj["spar_site"] = site
    if order is not None:
        obj["spar_patrol_order"] = int(order)
    return obj


def add_ordered_sites(
    name_prefix: str,
    site_prefix: str,
    points: Sequence[Sequence[float]],
    *,
    face: str = "next",
    headings: Sequence[float] | None = None,
    display_size: float = 1.0,
) -> list[bpy.types.Object]:
    """Create consistently named and ordered sites from XYZ route points."""
    route = [Vector(_as_vector(point, 3)) for point in points]
    if not route:
        raise ValueError("points must not be empty")
    if face not in {"next", "arrival"}:
        raise ValueError("face must be 'next' or 'arrival'")
    if headings is not None:
        yaws = [float(value) for value in headings]
        if len(yaws) != len(route) or not all(math.isfinite(value) for value in yaws):
            raise ValueError("headings must contain one finite yaw per point")
    else:
        yaws = route_headings(route, face=face)
    result = []
    for index, point in enumerate(route):
        result.append(
            add_site(
                f"{name_prefix}_{index + 1:02d}",
                f"{site_prefix}_{index + 1:02d}",
                point,
                rotation=(0.0, 0.0, yaws[index]),
                order=index,
                display_size=display_size,
            )
        )
    return result


def yaw_toward(origin: Sequence[float], target: Sequence[float]) -> float:
    """Return a planar yaw that points from origin to target."""
    start = Vector(_as_vector(origin, 3))
    end = Vector(_as_vector(target, 3))
    direction = end - start
    if direction.x * direction.x + direction.y * direction.y <= 1e-12:
        raise ValueError("origin and target need distinct XY positions")
    return math.atan2(direction.y, direction.x)


def route_headings(
    points: Sequence[Sequence[float]], *, face: str = "next"
) -> list[float]:
    """Return ordinary travel headings for a route without creating sites."""
    route = [Vector(_as_vector(point, 3)) for point in points]
    if not route:
        raise ValueError("points must not be empty")
    if face not in {"next", "arrival"}:
        raise ValueError("face must be 'next' or 'arrival'")
    headings = []
    for index, point in enumerate(route):
        if face == "arrival" and index > 0:
            direction = point - route[index - 1]
        elif index + 1 < len(route):
            direction = route[index + 1] - point
        elif index > 0:
            direction = point - route[index - 1]
        else:
            direction = Vector((1.0, 0.0, 0.0))
        headings.append(math.atan2(direction.y, direction.x))
    return headings


def _root_world_bounds(root: bpy.types.Object) -> tuple[Vector, Vector]:
    objects = [root, *root.children_recursive]
    corners = [
        obj.matrix_world @ Vector(corner)
        for obj in objects
        if obj.type == "MESH" and not obj.hide_render
        for corner in obj.bound_box
    ]
    if not corners:
        raise ValueError(f"target root has no visible mesh bounds: {root.name}")
    low = Vector(tuple(min(point[i] for point in corners) for i in range(3)))
    high = Vector(tuple(max(point[i] for point in corners) for i in range(3)))
    return low, high


def inspection_yaw(
    target_root: bpy.types.Object,
    location: Sequence[float],
    *,
    min_range: float = 3.0,
    max_range: float = 7.0,
    camera_height: float = 0.28,
) -> float:
    """Validate a ground-camera view of a target and return target-facing yaw."""
    point = Vector(_as_vector(location, 3))
    low, high = _root_world_bounds(target_root)
    target = (low + high) / 2.0
    planar_range = math.hypot(target.x - point.x, target.y - point.y)
    if not float(min_range) <= planar_range <= float(max_range):
        raise ValueError(
            f"inspection viewpoint range {planar_range:.3g} m is outside "
            f"[{float(min_range):.3g}, {float(max_range):.3g}] m"
        )

    eye = Vector((point.x, point.y, point.z + float(camera_height)))
    ray = target - eye
    distance = ray.length
    if distance <= 1e-8:
        raise ValueError("inspection viewpoint coincides with target")
    hit, _, _, _, hit_obj, _ = bpy.context.scene.ray_cast(
        bpy.context.evaluated_depsgraph_get(), eye, ray.normalized(),
        distance=distance + 1e-4,
    )
    if not hit or hit_obj is None:
        raise ValueError("inspection viewpoint ray did not reach the target")
    hit_obj = getattr(hit_obj, "original", hit_obj)
    current = hit_obj
    while current is not None and current != target_root:
        current = current.parent
    if current != target_root:
        raise ValueError(
            f"inspection viewpoint is occluded by {hit_obj.name}"
        )
    return yaw_toward(point, target)


def semantic_root(
    anchor: bpy.types.Object,
    *,
    spar_type: str | None = None,
    collection: str = "PROPS",
) -> bpy.types.Object:
    """Create an assembly root whose world transform exactly matches an anchor."""
    anchor_type = str(anchor.get("spar_plan_type", "assembly"))
    if spar_type is not None and spar_type != anchor_type:
        raise ValueError(
            f"semantic root type must preserve anchor type {anchor_type!r}, "
            f"got {spar_type!r}"
        )
    root_name = f"ASM_{anchor.name}"
    _require_new_object_name(root_name)
    root = bpy.data.objects.new(root_name, None)
    ensure_collection(collection).objects.link(root)
    # Anchor transforms assigned earlier in the same Blender call may not have
    # reached the evaluated dependency graph yet.  Copy only after an update;
    # otherwise matrix_world can still be identity and collapse every root.
    bpy.context.view_layer.update()
    root.matrix_world = anchor.matrix_world.copy()
    root["spar_type"] = anchor_type
    root["spar_plan_anchor"] = anchor.name
    role = anchor.get("spar_plan_role")
    if role:
        root["spar_role"] = role
    return root


def parent_local(
    child: bpy.types.Object,
    root: bpy.types.Object,
    *,
    location: Sequence[float] = (0.0, 0.0, 0.0),
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
) -> bpy.types.Object:
    """Parent without preserving the child's old world transform."""
    child.parent = root
    child.matrix_parent_inverse = Matrix.Identity(4)
    child.location = _as_vector(location, 3)
    child.rotation_euler = _as_vector(rotation, 3)
    return child


def _finish_primitive(
    obj: bpy.types.Object,
    root: bpy.types.Object,
    name: str,
    location: Sequence[float],
    rotation: Sequence[float],
    collection: str,
    material: bpy.types.Material | None,
    no_collision: bool,
) -> bpy.types.Object:
    obj.name = name
    move_to_collection(obj, collection)
    parent_local(obj, root, location=location, rotation=rotation)
    if material is not None:
        obj.data.materials.append(material)
    if no_collision:
        obj["spar_no_collision"] = True
    return obj


def add_box_local(
    root: bpy.types.Object,
    name: str,
    location: Sequence[float],
    dimensions: Sequence[float],
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    collection: str = "PROPS",
    material: bpy.types.Material | None = None,
    no_collision: bool = False,
    bevel: float = 0.0,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cube_add(size=1.0)
    obj = bpy.context.active_object
    # A size=1 Blender cube already has one-meter dimensions.  Scale by the
    # requested complete dimensions directly; dividing here would halve every
    # box built through the helper.
    obj.scale = _as_vector(dimensions, 3)
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    if bevel > 0.0:
        modifier = obj.modifiers.new("SPAR_Bevel", "BEVEL")
        modifier.width = float(bevel)
        modifier.segments = 2
    return _finish_primitive(
        obj,
        root,
        name,
        location,
        rotation,
        collection,
        material,
        no_collision,
    )


def add_ground_from_boundary(
    root: bpy.types.Object,
    boundary: bpy.types.Object,
    *,
    name: str = "ground_slab_box",
    thickness: float = 0.05,
    material: bpy.types.Material | None = None,
) -> bpy.types.Object:
    """Build a ground slab using the boundary's full stored dimensions."""
    width, depth, _ = dimensions_from_anchor(boundary)
    if width <= 0.0 or depth <= 0.0 or thickness <= 0.0:
        raise ValueError("ground dimensions and thickness must be positive")
    return add_box_local(
        root,
        name,
        (0.0, 0.0, -float(thickness) / 2.0),
        (width, depth, float(thickness)),
        material=material,
    )


def add_cylinder_local(
    root: bpy.types.Object,
    name: str,
    location: Sequence[float],
    radius: float,
    depth: float,
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    vertices: int = 20,
    collection: str = "PROPS",
    material: bpy.types.Material | None = None,
    no_collision: bool = False,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=int(vertices), radius=float(radius), depth=float(depth)
    )
    return _finish_primitive(
        bpy.context.active_object,
        root,
        name,
        location,
        rotation,
        collection,
        material,
        no_collision,
    )


def add_cone_local(
    root: bpy.types.Object,
    name: str,
    location: Sequence[float],
    radius1: float,
    radius2: float,
    depth: float,
    *,
    rotation: Sequence[float] = (0.0, 0.0, 0.0),
    vertices: int = 20,
    collection: str = "PROPS",
    material: bpy.types.Material | None = None,
    no_collision: bool = False,
) -> bpy.types.Object:
    bpy.ops.mesh.primitive_cone_add(
        vertices=int(vertices),
        radius1=float(radius1),
        radius2=float(radius2),
        depth=float(depth),
    )
    return _finish_primitive(
        bpy.context.active_object,
        root,
        name,
        location,
        rotation,
        collection,
        material,
        no_collision,
    )


def material(
    name: str,
    color: Sequence[float],
    *,
    metallic: float = 0.0,
    roughness: float = 0.5,
) -> bpy.types.Material:
    mat = bpy.data.materials.get(name) or bpy.data.materials.new(name)
    rgba = _as_vector(color, 4)
    mat.diffuse_color = rgba
    mat.use_nodes = True
    node = mat.node_tree.nodes.get("Principled BSDF")
    if node is not None:
        node.inputs["Base Color"].default_value = rgba
        node.inputs["Metallic"].default_value = float(metallic)
        node.inputs["Roughness"].default_value = float(roughness)
    return mat


def assign_material(obj: bpy.types.Object, mat: bpy.types.Material) -> None:
    if obj.type != "MESH":
        return
    obj.data.materials.clear()
    obj.data.materials.append(mat)


def ensure_camera(
    name: str,
    location: Sequence[float],
    target: Sequence[float],
    *,
    lens: float = 45.0,
) -> bpy.types.Object:
    obj = bpy.data.objects.get(name)
    if obj is None:
        data = bpy.data.cameras.new(name)
        obj = bpy.data.objects.new(name, data)
        ensure_collection("CAMERAS").objects.link(obj)
    obj.location = _as_vector(location, 3)
    direction = Vector(_as_vector(target, 3)) - obj.location
    obj.rotation_euler = direction.to_track_quat("-Z", "Y").to_euler()
    obj.data.lens = float(lens)
    return obj


def scene_top_z() -> float:
    """Return the highest world-space corner of render-visible mesh geometry."""
    top = None
    for obj in bpy.data.objects:
        if obj.type != "MESH" or obj.hide_render:
            continue
        for corner in obj.bound_box:
            value = (obj.matrix_world @ Vector(corner)).z
            top = value if top is None else max(top, value)
    return 0.0 if top is None else float(top)


def set_collection_visibility(
    name: str, *, viewport: bool | None = None, render: bool | None = None
) -> None:
    collection = bpy.data.collections.get(name)
    if collection is None:
        return
    if viewport is not None:
        collection.hide_viewport = not viewport
    if render is not None:
        collection.hide_render = not render


def finish_viewport(
    *,
    camera: str = "CAM_Overview",
    shading: str = "SOLID",
    material_color: bool = True,
) -> None:
    scene = bpy.context.scene
    camera_obj = bpy.data.objects.get(camera)
    if camera_obj is not None:
        scene.camera = camera_obj
    for window in bpy.context.window_manager.windows:
        for area in window.screen.areas:
            if area.type != "VIEW_3D":
                continue
            space = area.spaces.active
            if camera_obj is not None and space.region_3d is not None:
                space.region_3d.view_perspective = "CAMERA"
            space.overlay.show_overlays = True
            space.overlay.show_relationship_lines = False
            space.shading.type = shading
            if shading == "SOLID" and material_color:
                space.shading.color_type = "MATERIAL"
            if shading == "MATERIAL":
                space.shading.use_scene_lights = True
                space.shading.use_scene_world = True
            area.tag_redraw()


def save_checkpoint(path: str) -> None:
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=str(destination), check_existing=False)


def render_camera(camera: str, path: str) -> None:
    scene = bpy.context.scene
    camera_obj = bpy.data.objects.get(camera)
    if camera_obj is None or camera_obj.type != "CAMERA":
        raise ValueError(f"missing camera: {camera}")
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    scene.camera = camera_obj
    scene.render.filepath = str(destination)
    bpy.ops.render.render(write_still=True)


def write_scene_manifest(path: str) -> None:
    scene = bpy.context.scene
    roots = roots_by_type()

    def root_materials(root: bpy.types.Object) -> list[str]:
        return sorted(
            {
                slot.material.name
                for obj in [root, *root.children_recursive]
                for slot in obj.material_slots
                if slot.material
            }
        )

    payload = {
        "blender_version": bpy.app.version_string,
        "render_engine": scene.render.engine,
        "site_dimensions": [40.0, 40.0],
        "objects": [
            {
                "name": obj.name,
                "spar_type": obj.get("spar_type"),
                "transform": [list(row) for row in obj.matrix_world],
                "dimensions": list(obj.dimensions),
                "materials": root_materials(obj),
                "custom_properties": {
                    key: obj[key]
                    for key in obj.keys()
                    if isinstance(obj[key], (str, int, float, bool))
                },
            }
            for obj in roots
        ],
        "cameras": {
            obj.name: [list(row) for row in obj.matrix_world]
            for obj in bpy.data.objects
            if obj.type == "CAMERA"
        },
    }
    destination = Path(path).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
