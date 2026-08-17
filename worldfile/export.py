#!/usr/bin/env python3
"""Turn an accepted BlenderMCP scene into visual assets and one Worldfile MJCF.

Run inside Blender:
  blender --background artifacts/worldgen/utility_depot_40_v2/final.blend \
    --python worldfile/export.py -- --world utility_depot_40_v2

Every renderable Blender object whose geometry spans three dimensions receives
an automatic convex-mesh collider unless it sets worldfile_no_collision=true.
"""

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Vector


REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from worldfile.georeference import (  # noqa: E402 - Blender script path setup
    family_georeference,
    valid_georeference,
)

WORLD_DIR = REPO / "sim" / "worlds"
WORLD_CONFIG_DIR = REPO / "ros" / "src" / "worldfile" / "config" / "worlds"
SITES_COLLECTION = "WORLDFILE_SITES"
CONVERTIBLE_TYPES = {"MESH", "CURVE", "SURFACE", "FONT", "META"}
GROUND_DETECTOR_RANGE_M = 8.0
GROUND_HEADING_TOLERANCE_RAD = math.radians(25.0)
GROUND_OBSERVATION_TRAVEL_M = 20.0
GROUND_SPAWN_CLEARANCE_M = 0.8
# Keep these dimensions synchronized with husky_kinematic_footprint in
# sim/robots/husky.xml.  The probe occupies this vertical interval when the
# Husky root is at its authored ground height.
GROUND_FOOTPRINT_HALF_LENGTH_M = 0.52
GROUND_FOOTPRINT_HALF_WIDTH_M = 0.36
GROUND_FOOTPRINT_TURN_RADIUS_M = math.hypot(
    GROUND_FOOTPRINT_HALF_LENGTH_M, GROUND_FOOTPRINT_HALF_WIDTH_M)
GROUND_FOOTPRINT_LOW_M = 0.13228
GROUND_FOOTPRINT_HIGH_M = 0.33228
SITE_SIZE_M = 40.0
SITE_SIZE_TOLERANCE_M = 0.01


def arguments():
    command = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--world", default="utility_depot_40_v2")
    parser.add_argument(
        "--navigation-goals", action="store_true",
        help="export Blender-authored Nav2 demonstration goals",
    )
    return parser.parse_args(command)


def semantic_root(obj):
    current = obj
    while current:
        if isinstance(current.get("worldfile_type"), str):
            return current
        current = current.parent
    return None


def semantic_type(obj):
    root = semantic_root(obj)
    return root.get("worldfile_type") if root else None


def collision_source_metadata(obj):
    root = semantic_root(obj)
    source_root = root or obj
    return {
        "source_root": source_root.name,
        "source_type": root.get("worldfile_type") if root else "untyped_physical",
        "source_role": (
            root.get("worldfile_role") if root else obj.get("worldfile_role")
        ),
    }


def semantic_roots():
    return sorted(
        (obj for obj in bpy.context.scene.objects if isinstance(obj.get("worldfile_type"), str)),
        key=lambda obj: obj.name,
    )


def safe_name(value):
    value = re.sub(r"[^A-Za-z0-9_]+", "_", value).strip("_").lower()
    return value or "unnamed"


def portable_path(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def material_for_triangle(obj, triangle):
    if triangle.material_index < len(obj.material_slots):
        return obj.material_slots[triangle.material_index].material
    return None


def visual_objects():
    return [
        obj for obj in bpy.context.scene.objects
        if obj.type in CONVERTIBLE_TYPES
        and not obj.hide_render
        and semantic_type(obj) != "ground"
    ]


def collect_triangles():
    """Collect evaluated, world-space triangles in one chunk per material."""
    depsgraph = bpy.context.evaluated_depsgraph_get()
    chunks = defaultdict(lambda: {"material": None, "corners": []})
    for obj in visual_objects():
        evaluated = obj.evaluated_get(depsgraph)
        try:
            mesh = bpy.data.meshes.new_from_object(
                evaluated, preserve_all_data_layers=True, depsgraph=depsgraph)
        except RuntimeError as exc:
            print(f"[blender-export] skipping {obj.name}: {exc}")
            continue
        if not mesh.polygons:
            bpy.data.meshes.remove(mesh)
            continue
        mesh.calc_loop_triangles()
        uv_data = mesh.uv_layers.active.data if mesh.uv_layers.active else None
        normal_matrix = obj.matrix_world.to_3x3().inverted_safe().transposed()
        for triangle in mesh.loop_triangles:
            material = material_for_triangle(obj, triangle)
            key = material.name if material else "Worldfile_Default"
            chunk = chunks[key]
            chunk["material"] = material
            corners = []
            for loop_index in triangle.loops:
                vertex_index = mesh.loops[loop_index].vertex_index
                position = obj.matrix_world @ mesh.vertices[vertex_index].co
                uv = uv_data[loop_index].uv.copy() if uv_data else Vector((0, 0))
                normal = normal_matrix @ mesh.corner_normals[loop_index].vector
                normal.normalize()
                corners.append((tuple(position), tuple(uv), tuple(normal)))
            chunk["corners"].append(corners)
        bpy.data.meshes.remove(mesh)
    return chunks


def write_obj(path, triangles):
    """Write indexed positions, UVs, and authored split normals."""
    positions = {}
    texcoords = {}
    normals = {}
    faces = []
    for triangle in triangles:
        face = []
        for position, uv, normal in triangle:
            position_index = positions.setdefault(position, len(positions) + 1)
            texcoord_index = texcoords.setdefault(uv, len(texcoords) + 1)
            normal_index = normals.setdefault(normal, len(normals) + 1)
            face.append((position_index, texcoord_index, normal_index))
        faces.append(face)

    with path.open("w", encoding="utf-8") as output:
        output.write("# Generated from the accepted BlenderMCP scene.\n")
        for position in positions:
            output.write(
                f"v {position[0]:.7g} {position[1]:.7g} {position[2]:.7g}\n")
        for uv in texcoords:
            output.write(f"vt {uv[0]:.7g} {uv[1]:.7g}\n")
        for normal in normals:
            output.write(
                f"vn {normal[0]:.7g} {normal[1]:.7g} {normal[2]:.7g}\n")
        for face in faces:
            output.write(
                "f " + " ".join(f"{v}/{vt}/{vn}" for v, vt, vn in face) + "\n")


def automatic_collision_objects():
    """Renderable volumetric objects collide unless explicitly opted out."""
    # DevNote: Keep one collider per authored element. Material batching can
    # bridge unrelated props, and MuJoCo convex hulls fill openings unless the
    # opening is assembled from separate elements. Opt out only decoration.
    result = []
    for obj in visual_objects():
        if obj.get("worldfile_no_collision") is True:
            continue
        result.append(obj)
    return sorted(result, key=lambda item: item.name)


def collision_opt_outs():
    result = []
    for obj in visual_objects():
        if obj.get("worldfile_no_collision") is not True:
            continue
        result.append({
            "name": obj.name,
            "reason": "worldfile_no_collision",
            **collision_source_metadata(obj),
        })
    return sorted(result, key=lambda item: item["name"])


def has_volume(vertices):
    """Return whether points span 3D space, as required for a convex hull."""
    if len(vertices) < 4:
        return False
    points = [Vector(vertex) for vertex in vertices]
    span = max((point - points[0]).length for point in points)
    if span <= 1e-8:
        return False
    tolerance = span ** 3 * 1e-10
    first = next(
        (point for point in points[1:] if (point - points[0]).length > span * 1e-7),
        None,
    )
    if first is None:
        return False
    axis = first - points[0]
    second = next(
        (point for point in points[1:]
         if axis.cross(point - points[0]).length > span * span * 1e-8),
        None,
    )
    if second is None:
        return False
    normal = axis.cross(second - points[0])
    return any(
        abs(normal.dot(point - points[0])) > tolerance
        for point in points[1:]
    )


def collect_collision_mesh(obj, depsgraph):
    evaluated = obj.evaluated_get(depsgraph)
    try:
        mesh = bpy.data.meshes.new_from_object(
            evaluated, preserve_all_data_layers=False, depsgraph=depsgraph)
    except RuntimeError as exc:
        return None, f"evaluation failed: {exc}"
    try:
        if not mesh.polygons:
            return None, "no polygon faces"
        mesh.calc_loop_triangles()
        vertices = [
            tuple(evaluated.matrix_world @ vertex.co)
            for vertex in mesh.vertices
        ]
        faces = [tuple(triangle.vertices) for triangle in mesh.loop_triangles]
        if not faces:
            return None, "no triangulated faces"
        if not has_volume(vertices):
            return None, "mesh has no enclosed 3D volume"
        low = Vector(tuple(min(vertex[i] for vertex in vertices) for i in range(3)))
        high = Vector(tuple(max(vertex[i] for vertex in vertices) for i in range(3)))
        return {
            "vertices": vertices,
            "faces": faces,
            "position": (low + high) / 2,
            "size": (high - low) / 2,
        }, None
    finally:
        bpy.data.meshes.remove(mesh)


def write_collision_obj(path, vertices, faces):
    with path.open("w", encoding="utf-8") as output:
        output.write("# Per-element convex collision source from Blender.\n")
        for vertex in vertices:
            output.write(f"v {vertex[0]:.7g} {vertex[1]:.7g} {vertex[2]:.7g}\n")
        for face in faces:
            output.write("f " + " ".join(str(index + 1) for index in face) + "\n")


def color_image(material):
    if material is None or not material.use_nodes:
        return None
    candidates = []
    for node in material.node_tree.nodes:
        if node.type != "TEX_IMAGE" or node.image is None:
            continue
        name = f"{node.name} {node.label} {node.image.name}".lower()
        if any(word in name for word in ("normal", "rough", "displace", "height", "ao")):
            continue
        score = sum(word in name for word in ("diff", "albedo", "basecolor", "base_color"))
        candidates.append((score, node.image.name, node.image))
    return max(candidates, default=(0, "", None))[2]


def save_image(image, path):
    old_path = image.filepath_raw
    old_format = image.file_format
    try:
        image.filepath_raw = str(path)
        image.file_format = "PNG"
        image.save()
    finally:
        image.filepath_raw = old_path
        image.file_format = old_format


def rgba(material):
    color = material.diffuse_color if material else (0.55, 0.55, 0.55, 1)
    return " ".join(f"{max(0.0, min(1.0, value)):.4g}" for value in color)


def unique_names(values):
    result = {}
    used = set()
    for value in values:
        base = re.sub(r"^(?:mat|material)_", "", safe_name(value))
        base = base or "unnamed"
        name = base
        suffix = 2
        while name in used:
            name = f"{base}_{suffix}"
            suffix += 1
        used.add(name)
        result[value] = name
    return result


def ground_material():
    for root in semantic_roots():
        if root["worldfile_type"] != "ground":
            continue
        for obj in [root, *root.children_recursive]:
            for slot in obj.material_slots:
                if slot.material:
                    return slot.material
    return None


def site_bounds():
    depsgraph = bpy.context.evaluated_depsgraph_get()
    points = []
    for root in semantic_roots():
        if root["worldfile_type"] != "ground":
            continue
        for obj in [root, *root.children_recursive]:
            if obj.type not in CONVERTIBLE_TYPES or obj.hide_render:
                continue
            evaluated = obj.evaluated_get(depsgraph)
            points.extend(
                evaluated.matrix_world @ Vector(corner)
                for corner in evaluated.bound_box
            )
    if not points:
        raise RuntimeError("the scene has no visible worldfile_type='ground' geometry")
    low = Vector(tuple(min(point[i] for point in points) for i in range(3)))
    high = Vector(tuple(max(point[i] for point in points) for i in range(3)))
    if min(high.x - low.x, high.y - low.y) <= 0:
        raise RuntimeError("the ground has invalid horizontal bounds")
    return low, high


def quaternion_text(quaternion):
    quaternion.normalize()
    return f"{quaternion.w:.8g} {quaternion.x:.8g} {quaternion.y:.8g} {quaternion.z:.8g}"


def vector_text(vector):
    return " ".join(f"{value:.8g}" for value in vector)


def add_visual_assets(asset, worldbody, chunks, assets_dir):
    names = unique_names(chunks.keys())
    exported = []
    saved_images = {}
    for material_name, chunk in sorted(chunks.items()):
        if not chunk["corners"]:
            continue
        name = names[material_name]
        obj_path = assets_dir / f"mesh_{name}.obj"
        write_obj(obj_path, chunk["corners"])
        ET.SubElement(
            asset, "mesh", name=f"mesh_{name}", file=obj_path.name,
        )

        material = chunk["material"]
        attributes = {"name": f"mat_{name}", "rgba": rgba(material)}
        image = color_image(material)
        if image:
            image_key = image.name
            if image_key not in saved_images:
                texture_name = f"tex_{safe_name(image.name)}"
                texture_path = assets_dir / f"{texture_name}.png"
                save_image(image, texture_path)
                ET.SubElement(
                    asset, "texture", type="2d", name=texture_name,
                    file=texture_path.name,
                )
                saved_images[image_key] = texture_name
            attributes["texture"] = saved_images[image_key]
        ET.SubElement(asset, "material", **attributes)
        ET.SubElement(
            worldbody, "geom", name=f"vis_{name}", type="mesh",
            mesh=f"mesh_{name}", material=f"mat_{name}",
            **{"class": "worldfile_world_visual"},
        )
        exported.append(obj_path.name)
    return exported, [f"{name}.png" for name in saved_images.values()]


def add_ground(asset, worldbody, assets_dir):
    material = ground_material()
    attributes = {
        "name": "mat_ground",
        "rgba": rgba(material),
        "reflectance": "0.05",
    }
    image = color_image(material)
    texture_file = None
    if image:
        texture_file = "ground_color.png"
        save_image(image, assets_dir / texture_file)
        ET.SubElement(
            asset, "texture", type="2d", name="tex_ground",
            file=texture_file,
        )
        attributes.update(texture="tex_ground", texuniform="true", texrepeat="8 8")
    ET.SubElement(asset, "material", **attributes)
    low, high = site_bounds()
    center = (low + high) / 2
    dimensions = (high.x - low.x, high.y - low.y)
    if any(
        not math.isclose(value, SITE_SIZE_M, abs_tol=SITE_SIZE_TOLERANCE_M)
        for value in dimensions
    ):
        raise RuntimeError(
            f"generated worlds must be {SITE_SIZE_M:g} by {SITE_SIZE_M:g} m; "
            f"ground is {dimensions[0]:.3g} by {dimensions[1]:.3g} m")
    if abs(center.x) > SITE_SIZE_TOLERANCE_M or abs(center.y) > SITE_SIZE_TOLERANCE_M:
        raise RuntimeError("generated-world ground must be centered at the origin")
    ET.SubElement(
        worldbody, "geom", name="floor", type="plane",
        size=vector_text(Vector((dimensions[0] / 2, dimensions[1] / 2, 1))),
        pos=vector_text(Vector((center.x, center.y, high.z))),
        material="mat_ground",
        **{"class": "worldfile_world_ground"},
    )
    return texture_file, list(dimensions)


def add_collision_geoms(asset, worldbody, assets_dir):
    objects = automatic_collision_objects()
    names = unique_names(obj.name for obj in objects)
    depsgraph = bpy.context.evaluated_depsgraph_get()
    exported = []
    mesh_files = []
    omissions = []
    for obj in objects:
        collision_mesh, reason = collect_collision_mesh(obj, depsgraph)
        metadata = collision_source_metadata(obj)
        if collision_mesh is None:
            omissions.append({
                "name": obj.name,
                "source_root": metadata["source_root"],
                "source_type": metadata["source_type"],
                "reason": reason,
            })
            continue
        name = names[obj.name]
        mesh_name = f"colmesh_{name}"
        mesh_path = assets_dir / f"{mesh_name}.obj"
        write_collision_obj(
            mesh_path, collision_mesh["vertices"], collision_mesh["faces"])
        ET.SubElement(asset, "mesh", name=mesh_name, file=mesh_path.name)
        ET.SubElement(
            worldbody, "geom", name=f"col_auto_{name}", type="mesh",
            mesh=mesh_name, **{"class": "worldfile_world_collision"},
        )
        mesh_files.append(mesh_path.name)
        exported.append({
            "name": obj.name,
            **metadata,
            "position": list(collision_mesh["position"]),
            "size": list(collision_mesh["size"]),
            "mesh": mesh_path.name,
        })
    return exported, mesh_files, omissions


def add_sites(worldbody):
    sites_collection = bpy.data.collections.get(SITES_COLLECTION)
    if sites_collection is None:
        raise RuntimeError(f"scene is missing the required {SITES_COLLECTION} collection")
    exported = []
    for obj in sorted(sites_collection.objects, key=lambda item: item.name):
        role = obj.get("worldfile_site")
        if not isinstance(role, str):
            continue
        location = obj.matrix_world.translation
        yaw = obj.matrix_world.to_euler("XYZ").z
        if not all(math.isfinite(value) for value in (*location, yaw)):
            raise RuntimeError(f"{obj.name} has an invalid transform")
        ET.SubElement(
            worldbody, "site", name=safe_name(obj.name),
            pos=vector_text(location),
            quat=quaternion_text(obj.matrix_world.to_quaternion()),
            **{"class": "worldfile_world_site"},
        )
        exported.append({
            "name": obj.name,
            "role": role,
            "position": list(location),
            "yaw": yaw,
            "order": obj.get("worldfile_order"),
        })
    return exported


def ordered_navigation_sites(sites):
    goals = [site for site in sites
             if site["role"].startswith("navigation_goal_")]
    if len(goals) < 3:
        raise RuntimeError("--navigation-goals needs at least three goal sites")
    orders = [site["order"] for site in goals]
    if any(not isinstance(order, int) or isinstance(order, bool) for order in orders):
        raise RuntimeError("every navigation goal needs an integer worldfile_order")
    goals.sort(key=lambda site: site["order"])
    orders = [site["order"] for site in goals]
    if len(set(orders)) != len(orders):
        raise RuntimeError("navigation goal worldfile_order values must be unique")
    return goals


def ros_float(value):
    text = f"{float(value):.8g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def one_site(sites, role):
    matches = [site for site in sites if site["role"] == role]
    if len(matches) != 1:
        raise RuntimeError(f"the world needs exactly one {role} site")
    return matches[0]


def navigation_goals_in_world_frame(sites):
    spawn = one_site(sites, "husky_spawn")
    goals = []
    for site in ordered_navigation_sites(sites):
        goals.append({
            "name": safe_name(site["name"]),
            "order": int(site["order"]),
            "world_position": site["position"],
            "x": site["position"][0],
            "y": site["position"][1],
            "yaw": normalize_angle(site["yaw"]),
        })
    return spawn, goals


def validate_navigation_goals(spawn, goals, colliders, site_dimensions):
    half_x = site_dimensions[0] / 2.0
    half_y = site_dimensions[1] / 2.0
    for goal in goals:
        if (
            abs(goal["x"]) > half_x - GROUND_SPAWN_CLEARANCE_M
            or abs(goal["y"]) > half_y - GROUND_SPAWN_CLEARANCE_M
        ):
            raise RuntimeError(f"{goal['name']} is outside the safe site boundary")
        obstructions = []
        for collider in colliders:
            if (
                collider["position"][2] + collider["size"][2]
                <= GROUND_FOOTPRINT_LOW_M
                or collider["position"][2] - collider["size"][2]
                >= GROUND_FOOTPRINT_HIGH_M
            ):
                continue
            dx = max(
                abs(collider["position"][0] - goal["x"])
                - collider["size"][0],
                0.0,
            )
            dy = max(
                abs(collider["position"][1] - goal["y"])
                - collider["size"][1],
                0.0,
            )
            if math.hypot(dx, dy) < GROUND_FOOTPRINT_TURN_RADIUS_M:
                obstructions.append(collider["name"])
        if obstructions:
            raise RuntimeError(
                f"{goal['name']} obstructs the Husky turning footprint with "
                + ", ".join(obstructions)
            )
    target_colliders = [
        collider for collider in colliders
        if collider.get("source_role") == "anomaly"
        or collider.get("source_type") == "inspection_target"
    ]
    target_roots = {
        collider["source_root"] for collider in target_colliders
    }
    if len(target_roots) != 1:
        raise RuntimeError(
            "the navigation goals need exactly one inspection target root")
    low = [
        min(collider["position"][axis] - collider["size"][axis]
            for collider in target_colliders)
        for axis in range(3)
    ]
    high = [
        max(collider["position"][axis] + collider["size"][axis]
            for collider in target_colliders)
        for axis in range(3)
    ]
    target = [(low[axis] + high[axis]) / 2.0 for axis in range(3)]

    previous = spawn["position"]
    travel = 0.0
    observations = []
    for goal in goals:
        point = goal["world_position"]
        travel += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
        detector_range = math.hypot(
            target[0] - point[0], target[1] - point[1])
        target_yaw = math.atan2(target[1] - point[1], target[0] - point[0])
        heading_error = abs(normalize_angle(goal["yaw"] - target_yaw))
        if (
            detector_range <= GROUND_DETECTOR_RANGE_M
            and heading_error <= GROUND_HEADING_TOLERANCE_RAD
        ):
            observations.append({
                "goal": goal["name"],
                "order": goal["order"],
                "range_m": detector_range,
                "heading_error_rad": heading_error,
                "route_travel_m": travel,
            })
    if not observations:
        raise RuntimeError(
            "the navigation goals never put the red target in detector range "
            "and camera heading")
    first = min(observations, key=lambda item: item["route_travel_m"])
    if first["route_travel_m"] > GROUND_OBSERVATION_TRAVEL_M:
        raise RuntimeError(
            "the first observable red target needs "
            f"{first['route_travel_m']:.3g} m of route travel; limit is "
            f"{GROUND_OBSERVATION_TRAVEL_M:g} m"
        )
    return {
        "frame": "map_world_enu",
        "goals": goals,
        "first_observation": first,
        "detector_range_limit_m": GROUND_DETECTOR_RANGE_M,
        "heading_tolerance_rad": GROUND_HEADING_TOLERANCE_RAD,
        "observation_travel_limit_m": GROUND_OBSERVATION_TRAVEL_M,
        "line_of_sight_checked_in_blender": True,
        "goal_footprints_clear": True,
    }


def write_navigation_config(
        world, sites, colliders, site_dimensions, georeference, enabled):
    path = WORLD_CONFIG_DIR / f"{world}.yaml"
    if not enabled:
        if path.exists():
            path.unlink()
        return None, None

    spawn, goals = navigation_goals_in_world_frame(sites)
    dock = one_site(sites, "dock")
    metadata = validate_navigation_goals(
        spawn, goals, colliders, site_dimensions)
    lines = [
        "navsat_datum: ["
        f"{ros_float(georeference['latitude_deg'])}, "
        f"{ros_float(georeference['longitude_deg'])}, 0.0]",
        "dock_pose: ["
        f"{ros_float(dock['position'][0])}, "
        f"{ros_float(dock['position'][1])}, "
        f"{ros_float(dock['yaw'])}]",
        "navigation_goals:",
    ]
    for goal in goals:
        lines.append(
            "  - {name: " + goal["name"]
            + f", x: {ros_float(goal['x'])}, y: {ros_float(goal['y'])}, "
            + f"yaw: {ros_float(goal['yaw'])}}}"
        )
    WORLD_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n")
    return str(path.relative_to(REPO)), metadata


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def validate_ground_sites(sites, colliders, site_dimensions):
    husky = one_site(sites, "husky_spawn")
    dock = one_site(sites, "dock")
    for site in (husky, dock):
        x, y, _ = site["position"]
        if (
            abs(x) > site_dimensions[0] / 2.0 - GROUND_SPAWN_CLEARANCE_M
            or abs(y) > site_dimensions[1] / 2.0 - GROUND_SPAWN_CLEARANCE_M
        ):
            raise RuntimeError(
                f"{site['role']} is outside the safe site boundary")
        if any(
            abs(collider["position"][0] - x)
            < collider["size"][0] + GROUND_SPAWN_CLEARANCE_M
            and abs(collider["position"][1] - y)
            < collider["size"][1] + GROUND_SPAWN_CLEARANCE_M
            for collider in colliders
        ):
            raise RuntimeError(
                f"{site['role']} ground clearance is obstructed")


def add_overview_camera(worldbody):
    camera = bpy.data.objects.get("CAM_Overview")
    if camera is None or camera.type != "CAMERA":
        return
    location, rotation, _ = camera.matrix_world.decompose()
    ET.SubElement(
        worldbody, "camera", name="overview", pos=vector_text(location),
        quat=quaternion_text(rotation), fovy=f"{math.degrees(camera.data.angle_y):.6g}",
    )


def add_robot(asset, worldbody, sites):
    """Attach the pose-neutral Husky model at its world-authored spawn."""
    ET.SubElement(
        asset, "model", name="husky_model", file="../robots/husky.xml"
    )
    site = one_site(sites, "husky_spawn")
    half_yaw = site["yaw"] / 2.0
    frame = ET.SubElement(
        worldbody,
        "frame",
        pos=" ".join(ros_float(value) for value in site["position"]),
        quat=f"{ros_float(math.cos(half_yaw))} 0 0 "
             f"{ros_float(math.sin(half_yaw))}",
    )
    ET.SubElement(frame, "attach", model="husky_model", body="base_link", prefix="")

    dock = one_site(sites, "dock")
    ET.SubElement(
        worldbody, "geom", name="dock_pad", type="box", size="0.9 0.9 0.005",
        pos=(f"{ros_float(dock['position'][0])} "
             f"{ros_float(dock['position'][1])} 0.005"),
        contype="0", conaffinity="0", group="1", rgba="0.2 0.4 0.8 1",
    )


def build_mjcf(world, assets_dir, world_path, georeference):
    root = ET.Element("mujoco", model=world)
    ET.SubElement(
        root, "compiler", angle="radian", assetdir=f"assets/{world}",
    )
    custom = ET.SubElement(root, "custom")
    ET.SubElement(
        custom,
        "numeric",
        name="worldfile.world_georeference",
        data=" ".join(
            ros_float(georeference[key])
            for key in ("latitude_deg", "longitude_deg", "altitude_m")
        ),
    )
    defaults = ET.SubElement(root, "default")
    visual_defaults = ET.SubElement(
        defaults, "default", **{"class": "worldfile_world_visual"})
    ET.SubElement(
        visual_defaults, "geom", contype="0", conaffinity="0", group="2")
    collision_defaults = ET.SubElement(
        defaults, "default", **{"class": "worldfile_world_collision"})
    ET.SubElement(
        collision_defaults, "geom", contype="1", conaffinity="1",
        condim="3", group="3", friction="1 0.005 0.0001",
        rgba="0 0 0 0",
    )
    ground_defaults = ET.SubElement(
        defaults, "default", **{"class": "worldfile_world_ground"})
    ET.SubElement(
        ground_defaults, "geom", contype="1", conaffinity="1",
        condim="3", group="4", friction="1 0.005 0.0001",
    )
    site_defaults = ET.SubElement(
        defaults, "default", **{"class": "worldfile_world_site"})
    ET.SubElement(
        site_defaults, "site", size="0.05", group="5",
        rgba="0.2 0.8 1 0.6",
    )
    ET.SubElement(
        root, "option", timestep="0.002", integrator="implicitfast",
        magnetic="0 0 0",
    )
    visual = ET.SubElement(root, "visual")
    ET.SubElement(
        visual, "headlight", diffuse="0.6 0.6 0.6", ambient="0.3 0.3 0.3",
        specular="0 0 0",
    )
    ET.SubElement(
        visual, "global", azimuth="120", elevation="-20",
        offwidth="1280", offheight="720",
    )

    asset = ET.SubElement(root, "asset")
    ET.SubElement(
        asset, "texture", type="skybox", builtin="gradient",
        rgb1="0.55 0.65 0.75", rgb2="0.08 0.1 0.14",
        width="512", height="3072",
    )
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody, "light", name="sun", directional="true", pos="-4 -6 12",
        dir="0.25 0.35 -1", diffuse="0.9 0.88 0.82",
    )
    add_overview_camera(worldbody)
    ground_texture, site_dimensions = add_ground(asset, worldbody, assets_dir)
    chunks = collect_triangles()
    visual_meshes, visual_textures = add_visual_assets(
        asset, worldbody, chunks, assets_dir)
    colliders, collision_meshes, collision_omissions = add_collision_geoms(
        asset, worldbody, assets_dir)
    sites = add_sites(worldbody)
    validate_ground_sites(sites, colliders, site_dimensions)
    add_robot(asset, worldbody, sites)
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(world_path, encoding="unicode", xml_declaration=False)
    return {
        "world": world,
        "world_file": str(world_path.relative_to(REPO)),
        "georeference": georeference,
        "visual_meshes": visual_meshes,
        "collision_meshes": collision_meshes,
        "visual_textures": sorted(visual_textures),
        "ground_texture": ground_texture,
        "site_dimensions_m": site_dimensions,
        "colliders": colliders,
        "collision_opt_outs": collision_opt_outs(),
        "collision_omissions": collision_omissions,
        "sites": sites,
        "visual_triangles": sum(len(chunk["corners"]) for chunk in chunks.values()),
    }


def remove_stale_generated_assets(assets_dir, manifest):
    referenced = set(manifest["visual_meshes"])
    referenced.update(manifest.get("collision_meshes", []))
    referenced.update(manifest["visual_textures"])
    if manifest["ground_texture"]:
        referenced.add(manifest["ground_texture"])
    referenced.add("export_manifest.json")

    for path in assets_dir.iterdir():
        exporter_owned = (
            path.name.startswith("visual_")
            or path.name.startswith("mesh_")
            or path.name.startswith("colmesh_")
            or path.name.startswith("texture_")
            or path.name.startswith("tex_")
            or path.name == "ground_color.png"
        )
        if path.is_file() and exporter_owned and path.name not in referenced:
            path.unlink()


def add_generation_provenance(manifest, source_blend, world):
    recipe_path = source_blend.parent / "generation.json"
    if not recipe_path.is_file():
        return
    contents = recipe_path.read_bytes()
    try:
        recipe = json.loads(contents)
    except json.JSONDecodeError as error:
        raise RuntimeError(f"invalid generation recipe: {recipe_path}") from error
    if recipe.get("world") != world:
        raise RuntimeError("generation recipe world does not match --world")
    manifest["generation"] = recipe
    manifest["generation_sha256"] = hashlib.sha256(contents).hexdigest()


def export_georeference(source_blend, world):
    recipe_path = source_blend.parent / "generation.json"
    if not recipe_path.is_file():
        return family_georeference("utility_depot")
    try:
        recipe = json.loads(recipe_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise RuntimeError(f"invalid generation recipe: {recipe_path}") from error
    if recipe.get("world") != world:
        raise RuntimeError("generation recipe world does not match --world")
    georeference = recipe.get("georeference")
    if not valid_georeference(georeference):
        raise RuntimeError("generation recipe has an invalid georeference")
    return georeference


def main():
    args = arguments()
    if not bpy.data.filepath:
        raise SystemExit("save the BlenderMCP scene before exporting it")
    args.world = args.world.lower()
    if safe_name(args.world) != args.world:
        raise SystemExit("--world must contain only letters, numbers, and underscores")
    source_blend = Path(bpy.data.filepath).resolve()
    georeference = export_georeference(source_blend, args.world)

    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    assets_dir = WORLD_DIR / "assets" / args.world
    assets_dir.mkdir(parents=True, exist_ok=True)
    world_path = WORLD_DIR / f"{args.world}.xml"
    manifest = build_mjcf(args.world, assets_dir, world_path, georeference)
    manifest["source_blend"] = portable_path(source_blend)
    add_generation_provenance(manifest, source_blend, args.world)
    manifest["navigation_goal_mode"] = (
        "world" if args.navigation_goals else "none")
    navigation_config, navigation_goals = write_navigation_config(
        args.world, manifest["sites"], manifest["colliders"],
        manifest["site_dimensions_m"],
        manifest["georeference"],
        args.navigation_goals)
    manifest["navigation_config"] = navigation_config
    manifest["navigation_goals"] = navigation_goals
    manifest_path = assets_dir / "export_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    remove_stale_generated_assets(assets_dir, manifest)

    print(f"[blender-export] world: {world_path}")
    print(
        f"[blender-export] {len(manifest['visual_meshes'])} visual meshes, "
        f"{manifest['visual_triangles']} triangles, "
        f"{len(manifest['colliders'])} automatic colliders")
    if manifest["collision_omissions"]:
        print(
            f"[blender-export] {len(manifest['collision_omissions'])} "
            "non-volumetric collision candidates omitted")
    if manifest["navigation_config"]:
        print(
            f"[blender-export] navigation goals: "
            f"{REPO / manifest['navigation_config']}")
        print(
            f"[blender-export] launch: ros2 launch worldfile_demo "
            f"navigation.launch.py world:={args.world}")


if __name__ == "__main__":
    main()
