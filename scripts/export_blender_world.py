#!/usr/bin/env python3
"""Turn an accepted BlenderMCP scene into visual assets and one SPAR MJCF.

Run inside Blender:
  blender --background artifacts/worldgen/utility_depot_40_v1/final.blend \
    --python scripts/export_blender_world.py -- --world utility_depot_40_v1

The first run adds editable collision proxies and semantic sites, then saves
simulation.blend beside the source file. Later runs preserve those proxies so
they can be adjusted by hand or through BlenderMCP.
"""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Vector


REPO = Path(__file__).resolve().parents[1]
WORLD_DIR = REPO / "sim" / "worlds"
GROUND_WORLD_CONFIG_DIR = REPO / "ground" / "src" / "spar_ground" / "config" / "worlds"
AIR_WORLD_CONFIG_DIR = REPO / "air" / "src" / "spar_air" / "config" / "worlds"
VISUAL_COLLECTION = "SPAR_VISUAL"
COLLISION_COLLECTION = "SPAR_COLLISION"
SITES_COLLECTION = "SPAR_SITES"
CONVERTIBLE_TYPES = {"MESH", "CURVE", "SURFACE", "FONT", "META"}
BOX_TYPES = {"fence", "rack", "cable_spool", "utility_cabinet"}
CYLINDER_TYPES = {"barrel", "inspection_target"}
VISUAL_ONLY_TYPES = {"ground", "gate", "pallet"}
AIR_HORIZONTAL_CLEARANCE_M = 0.6
AIR_VERTICAL_CLEARANCE_M = 0.5
AIR_MAX_ALTITUDE_M = 10.0
AIR_DETECTOR_RANGE_M = 12.0
SITE_SIZE_M = 40.0
SITE_SIZE_TOLERANCE_M = 0.01


def arguments():
    command = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--world", default="utility_depot_40_v1")
    parser.add_argument(
        "--world-waypoints", action="store_true",
        help="export Blender-authored ground and air patrol waypoint files",
    )
    parser.add_argument(
        "--save-blend",
        help="prepared .blend path; defaults to simulation.blend beside the input",
    )
    return parser.parse_args(command)


def collection(name):
    result = bpy.data.collections.get(name)
    if result is None:
        result = bpy.data.collections.new(name)
        bpy.context.scene.collection.children.link(result)
    return result


def semantic_type(obj):
    current = obj
    while current:
        kind = current.get("spar_type")
        if isinstance(kind, str):
            return kind
        current = current.parent
    return None


def semantic_roots():
    return sorted(
        (obj for obj in bpy.context.scene.objects if isinstance(obj.get("spar_type"), str)),
        key=lambda obj: obj.name,
    )


def link_visual_objects():
    visuals = collection(VISUAL_COLLECTION)
    collision_objects = set(collection(COLLISION_COLLECTION).objects)
    for obj in bpy.context.scene.objects:
        if obj.type not in CONVERTIBLE_TYPES or obj.hide_render:
            continue
        if obj in collision_objects or obj.get("spar_collision_shape"):
            continue
        if obj.name not in visuals.objects:
            visuals.objects.link(obj)


def local_bounds(root, depsgraph):
    """Bounds of a semantic assembly in its root object's coordinate frame."""
    points = []
    to_local = root.matrix_world.inverted_safe()
    objects = [root, *root.children_recursive]
    for obj in objects:
        if obj.type not in CONVERTIBLE_TYPES or obj.hide_render:
            continue
        evaluated = obj.evaluated_get(depsgraph)
        for corner in evaluated.bound_box:
            points.append(to_local @ evaluated.matrix_world @ Vector(corner))
    if not points:
        return None
    low = Vector(tuple(min(point[i] for point in points) for i in range(3)))
    high = Vector(tuple(max(point[i] for point in points) for i in range(3)))
    if min(high - low) <= 0:
        return None
    return low, high


def cube_mesh(name):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(
        [
            (-0.5, -0.5, -0.5), (-0.5, -0.5, 0.5),
            (-0.5, 0.5, -0.5), (-0.5, 0.5, 0.5),
            (0.5, -0.5, -0.5), (0.5, -0.5, 0.5),
            (0.5, 0.5, -0.5), (0.5, 0.5, 0.5),
        ],
        [],
        [
            (0, 4, 6, 2), (1, 3, 7, 5), (0, 1, 5, 4),
            (2, 6, 7, 3), (0, 2, 3, 1), (4, 5, 7, 6),
        ],
    )
    return mesh


def cylinder_mesh(name, segments=24):
    vertices = []
    for z in (-0.5, 0.5):
        vertices.extend(
            (0.5 * math.cos(2 * math.pi * i / segments),
             0.5 * math.sin(2 * math.pi * i / segments), z)
            for i in range(segments)
        )
    faces = []
    for i in range(segments):
        next_i = (i + 1) % segments
        faces.append((i, next_i, segments + next_i, segments + i))
    faces.append(tuple(reversed(range(segments))))
    faces.append(tuple(range(segments, 2 * segments)))
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    return mesh


def add_proxy(name, shape, root, center, dimensions):
    proxies = collection(COLLISION_COLLECTION)
    mesh = cube_mesh(name) if shape == "box" else cylinder_mesh(name)
    obj = bpy.data.objects.new(name, mesh)
    proxies.objects.link(obj)
    obj.matrix_world = root.matrix_world @ Matrix.Translation(center)
    obj.scale = dimensions
    obj.display_type = "WIRE"
    obj.show_in_front = True
    obj.hide_render = True
    obj.color = (0.15, 1.0, 0.2, 1.0)
    obj["spar_collision_shape"] = shape
    obj["spar_source"] = root.name
    return obj


def add_structure_proxies(root, low, high):
    size = high - low
    center = (low + high) / 2
    roof_depth = min(0.18, size.z * 0.12)
    roof_center = Vector((center.x, center.y, high.z - roof_depth / 2))
    add_proxy(
        f"COL_{root.name}_Roof", "box", root, roof_center,
        Vector((size.x, size.y, roof_depth)),
    )

    post_width = min(0.16, size.x * 0.08, size.y * 0.08)
    post_height = max(size.z - roof_depth, 0.1)
    post_z = low.z + post_height / 2
    index = 1
    for x in (low.x + post_width / 2, high.x - post_width / 2):
        for y in (low.y + post_width / 2, high.y - post_width / 2):
            add_proxy(
                f"COL_{root.name}_Post_{index}", "box", root,
                Vector((x, y, post_z)),
                Vector((post_width, post_width, post_height)),
            )
            index += 1


def create_initial_proxies():
    proxies = collection(COLLISION_COLLECTION)
    if proxies.objects:
        return False

    depsgraph = bpy.context.evaluated_depsgraph_get()
    for root in semantic_roots():
        kind = root["spar_type"]
        if kind in VISUAL_ONLY_TYPES:
            root["spar_collision"] = "visual_only"
            continue
        bounds = local_bounds(root, depsgraph)
        if bounds is None:
            print(f"[blender-export] no visible bounds for {root.name}; visual only")
            root["spar_collision"] = "visual_only"
            continue
        low, high = bounds
        center = (low + high) / 2
        dimensions = high - low

        if kind == "structure":
            add_structure_proxies(root, low, high)
        elif kind in CYLINDER_TYPES:
            diameter = max(dimensions.x, dimensions.y)
            add_proxy(
                f"COL_{root.name}", "cylinder", root, center,
                Vector((diameter, diameter, dimensions.z)),
            )
        elif kind in BOX_TYPES:
            add_proxy(f"COL_{root.name}", "box", root, center, dimensions)
        else:
            root["spar_collision"] = "visual_only"
    return True


def add_empty(name, location, role, patrol_order=None):
    sites = collection(SITES_COLLECTION)
    obj = bpy.data.objects.get(name)
    if obj is None:
        obj = bpy.data.objects.new(name, None)
        sites.objects.link(obj)
        obj.location = location
    obj.empty_display_type = "ARROWS"
    obj.empty_display_size = 0.6
    obj["spar_site"] = role
    if patrol_order is not None:
        obj["spar_patrol_order"] = patrol_order


def create_sites():
    add_empty("SITE_HuskySpawn", (0, 0, 0), "husky_spawn")
    add_empty("SITE_Dock", (0, 0, 0), "dock")
    add_empty("SITE_X2Spawn", (-2, -1, 0), "x2_spawn")


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
    collisions = set(collection(COLLISION_COLLECTION).objects)
    return [
        obj for obj in collection(VISUAL_COLLECTION).all_objects
        if obj.type in CONVERTIBLE_TYPES
        and obj not in collisions
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
        for triangle in mesh.loop_triangles:
            material = material_for_triangle(obj, triangle)
            key = material.name if material else "SPAR_Default"
            chunk = chunks[key]
            chunk["material"] = material
            corners = []
            for loop_index in triangle.loops:
                vertex_index = mesh.loops[loop_index].vertex_index
                position = obj.matrix_world @ mesh.vertices[vertex_index].co
                uv = uv_data[loop_index].uv.copy() if uv_data else Vector((0, 0))
                corners.append((tuple(position), tuple(uv)))
            chunk["corners"].append(corners)
        bpy.data.meshes.remove(mesh)
    return chunks


def write_obj(path, triangles):
    with path.open("w", encoding="utf-8") as output:
        output.write("# Generated from the accepted BlenderMCP scene.\n")
        index = 1
        faces = []
        for triangle in triangles:
            face = []
            for position, uv in triangle:
                output.write(f"v {position[0]:.7g} {position[1]:.7g} {position[2]:.7g}\n")
                output.write(f"vt {uv[0]:.7g} {uv[1]:.7g}\n")
                face.append(index)
                index += 1
            faces.append(face)
        for face in faces:
            output.write("f " + " ".join(f"{i}/{i}" for i in face) + "\n")


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
        base = safe_name(value)
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
        if root["spar_type"] != "ground":
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
        if root["spar_type"] != "ground":
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
        raise RuntimeError("the scene has no visible spar_type='ground' geometry")
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
        obj_path = assets_dir / f"visual_{name}.obj"
        write_obj(obj_path, chunk["corners"])
        ET.SubElement(
            asset, "mesh", name=f"visual_{name}", file=obj_path.name,
            smoothnormal="true",
        )

        material = chunk["material"]
        attributes = {"name": f"visual_{name}", "rgba": rgba(material)}
        image = color_image(material)
        if image:
            image_key = image.name
            if image_key not in saved_images:
                texture_name = f"texture_{safe_name(image.name)}"
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
            worldbody, "geom", name=f"visual_{name}", type="mesh",
            mesh=f"visual_{name}", material=f"visual_{name}",
            contype="0", conaffinity="0", group="2", density="0",
        )
        exported.append(obj_path.name)
    return exported, [f"{name}.png" for name in saved_images.values()]


def add_ground(asset, worldbody, assets_dir):
    material = ground_material()
    attributes = {
        "name": "ground_material",
        "rgba": rgba(material),
        "reflectance": "0.05",
    }
    image = color_image(material)
    texture_file = None
    if image:
        texture_file = "ground_color.png"
        save_image(image, assets_dir / texture_file)
        ET.SubElement(
            asset, "texture", type="2d", name="ground_texture",
            file=texture_file,
        )
        attributes.update(texture="ground_texture", texuniform="true", texrepeat="8 8")
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
        material="ground_material",
    )
    return texture_file, list(dimensions)


def add_collision_geoms(worldbody):
    exported = []
    for obj in sorted(collection(COLLISION_COLLECTION).objects, key=lambda item: item.name):
        shape = obj.get("spar_collision_shape")
        if shape not in {"box", "cylinder"}:
            raise RuntimeError(f"{obj.name} has unsupported collision shape {shape!r}")
        location, rotation, scale = obj.matrix_world.decompose()
        scale = Vector(tuple(abs(value) for value in scale))
        if min(scale) <= 0 or not all(math.isfinite(value) for value in (*location, *scale)):
            raise RuntimeError(f"{obj.name} has an invalid transform")
        if shape == "box":
            size = scale / 2
        else:
            size = Vector((max(scale.x, scale.y) / 2, scale.z / 2))
        attributes = {
            "name": safe_name(obj.name),
            "type": shape,
            "size": vector_text(size),
            "pos": vector_text(location),
            "quat": quaternion_text(rotation),
            "group": "3",
            "rgba": "0 0 0 0",
        }
        ET.SubElement(worldbody, "geom", **attributes)
        source_name = obj.get("spar_source", "")
        source = bpy.data.objects.get(source_name) if source_name else None
        exported.append({
            "name": obj.name,
            "shape": shape,
            "source": source_name,
            "source_type": semantic_type(source) if source else None,
            "source_role": source.get("spar_role") if source else None,
            "position": list(location),
            "size": list(size),
        })
    return exported


def add_sites(worldbody):
    exported = []
    for obj in sorted(collection(SITES_COLLECTION).objects, key=lambda item: item.name):
        role = obj.get("spar_site")
        if not isinstance(role, str):
            continue
        location = obj.matrix_world.translation
        yaw = obj.matrix_world.to_euler("XYZ").z
        if not all(math.isfinite(value) for value in (*location, yaw)):
            raise RuntimeError(f"{obj.name} has an invalid transform")
        ET.SubElement(
            worldbody, "site", name=safe_name(obj.name), pos=vector_text(location),
            size="0.05", group="5", rgba="0.2 0.8 1 0.6",
        )
        exported.append({
            "name": obj.name,
            "role": role,
            "position": list(location),
            "yaw": yaw,
            "patrol_order": obj.get("spar_patrol_order"),
        })
    return exported


def ordered_patrol_sites(sites, prefix, label):
    patrol = [site for site in sites if site["role"].startswith(prefix)]
    if len(patrol) < 3:
        raise RuntimeError(
            f"--world-waypoints needs at least three {prefix}* sites")
    orders = [site["patrol_order"] for site in patrol]
    if any(not isinstance(order, int) or isinstance(order, bool) for order in orders):
        raise RuntimeError(
            f"every {label} patrol site needs an integer spar_patrol_order")
    patrol.sort(key=lambda site: site["patrol_order"])
    orders = [site["patrol_order"] for site in patrol]
    if len(set(orders)) != len(orders):
        raise RuntimeError(f"{label} spar_patrol_order values must be unique")
    return patrol


def ros_float(value):
    text = f"{float(value):.8g}"
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def write_ground_waypoint_config(world, sites, enabled):
    path = GROUND_WORLD_CONFIG_DIR / f"{world}.yaml"
    if not enabled:
        if path.exists():
            path.unlink()
        return None

    patrol = ordered_patrol_sites(sites, "patrol_", "ground")

    values = []
    for site in patrol:
        values.extend((site["position"][0], site["position"][1], site["yaw"]))

    # ROS parameter arrays must be homogeneous.  Explicit decimal points keep
    # whole-number coordinates from being parsed as integers beside yaw floats.
    formatted = ", ".join(ros_float(value) for value in values)
    GROUND_WORLD_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**/bt_executive:\n"
        "  ros__parameters:\n"
        f"    patrol_waypoints: [{formatted}]\n"
    )
    return str(path.relative_to(REPO))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def air_route_in_local_frame(sites):
    spawns = [site for site in sites if site["role"] == "x2_spawn"]
    if len(spawns) != 1:
        raise RuntimeError("the air route needs exactly one x2_spawn site")
    spawn = spawns[0]
    patrol = ordered_patrol_sites(sites, "air_patrol_", "air")
    spawn_yaw = spawn["yaw"]
    cos_yaw = math.cos(-spawn_yaw)
    sin_yaw = math.sin(-spawn_yaw)
    route = []
    for site in patrol:
        dx = site["position"][0] - spawn["position"][0]
        dy = site["position"][1] - spawn["position"][1]
        route.append({
            "name": site["name"],
            "order": int(site["patrol_order"]),
            "world_position": site["position"],
            "x": cos_yaw * dx - sin_yaw * dy,
            "y": sin_yaw * dx + cos_yaw * dy,
            "z": site["position"][2] - spawn["position"][2],
            "yaw": normalize_angle(site["yaw"] - spawn_yaw),
        })
    return spawn, route


def proxy_top(proxy):
    half_height = proxy["size"][1] if proxy["shape"] == "cylinder" else proxy["size"][2]
    return proxy["position"][2] + half_height


def proxy_overlaps_pad(proxy, pad_x, pad_y):
    distance = math.hypot(
        proxy["position"][0] - pad_x,
        proxy["position"][1] - pad_y,
    )
    if proxy["shape"] == "cylinder":
        radius = proxy["size"][0] + AIR_HORIZONTAL_CLEARANCE_M
    else:
        radius = math.hypot(proxy["size"][0], proxy["size"][1])
        radius += AIR_HORIZONTAL_CLEARANCE_M
    return distance < radius


def validate_air_route(spawn, route, collision_proxies, site_dimensions):
    world_safe_altitude = max(
        (proxy_top(proxy) for proxy in collision_proxies), default=0.0
    ) + AIR_VERTICAL_CLEARANCE_M
    local_safe_altitude = world_safe_altitude - spawn["position"][2]
    half_x = site_dimensions[0] / 2
    half_y = site_dimensions[1] / 2

    if any(proxy_overlaps_pad(
            proxy, spawn["position"][0], spawn["position"][1])
           for proxy in collision_proxies):
        raise RuntimeError(
            "the x2_spawn vertical takeoff and landing corridor is obstructed")

    for waypoint in route:
        world_x, world_y, _ = waypoint["world_position"]
        if (
            abs(world_x) > half_x - AIR_HORIZONTAL_CLEARANCE_M
            or abs(world_y) > half_y - AIR_HORIZONTAL_CLEARANCE_M
        ):
            raise RuntimeError(f"{waypoint['name']} is outside the air route bounds")
        if waypoint["z"] < local_safe_altitude:
            raise RuntimeError(
                f"{waypoint['name']} altitude {waypoint['z']:.3g} m is below "
                f"the {local_safe_altitude:.3g} m collision-clear flight floor")
        if waypoint["z"] > AIR_MAX_ALTITUDE_M:
            raise RuntimeError(
                f"{waypoint['name']} exceeds the {AIR_MAX_ALTITUDE_M:g} m air ceiling")

    anomalies = [
        proxy for proxy in collision_proxies
        if proxy.get("source_role") == "anomaly"
        or proxy.get("source_type") == "inspection_target"
    ]
    if len(anomalies) != 1:
        raise RuntimeError("the air route needs exactly one inspection target")
    anomaly = anomalies[0]
    closest = min(
        math.dist(waypoint["world_position"], anomaly["position"])
        for waypoint in route
    )
    if closest > AIR_DETECTOR_RANGE_M:
        raise RuntimeError(
            f"the closest air waypoint is {closest:.3g} m from the anomaly; "
            f"the detector limit is {AIR_DETECTOR_RANGE_M:g} m")
    return {
        "collision_clear_flight_floor_m": local_safe_altitude,
        "closest_anomaly_range_m": closest,
        "takeoff_corridor_clear": True,
    }


def write_air_waypoint_config(
        world, sites, collision_proxies, site_dimensions, enabled):
    path = AIR_WORLD_CONFIG_DIR / f"{world}.yaml"
    if not enabled:
        if path.exists():
            path.unlink()
        return None, None

    spawn, route = air_route_in_local_frame(sites)
    safety = validate_air_route(spawn, route, collision_proxies, site_dimensions)
    cruise_altitude = max(waypoint["z"] for waypoint in route)
    values = []
    for waypoint in route:
        values.extend((waypoint["x"], waypoint["y"], waypoint["z"], waypoint["yaw"]))
    formatted = ", ".join(ros_float(value) for value in values)
    AIR_WORLD_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    path.write_text(
        "/**/bt_executive:\n"
        "  ros__parameters:\n"
        f"    cruise_alt_m: {ros_float(cruise_altitude)}\n"
        f"    orbit_alt_m: {ros_float(cruise_altitude)}\n"
        f"    patrol_waypoints: [{formatted}]\n"
    )
    return str(path.relative_to(REPO)), {
        "frame": "x2_spawn_local_enu",
        "waypoints": route,
        **safety,
    }


def add_overview_camera(worldbody):
    camera = bpy.data.objects.get("CAM_Overview")
    if camera is None or camera.type != "CAMERA":
        return
    location, rotation, _ = camera.matrix_world.decompose()
    ET.SubElement(
        worldbody, "camera", name="overview", pos=vector_text(location),
        quat=quaternion_text(rotation), fovy=f"{math.degrees(camera.data.angle_y):.6g}",
    )


def build_mjcf(world, assets_dir, world_path):
    root = ET.Element("mujoco", model=world)
    ET.SubElement(
        root, "compiler", angle="radian", meshdir=f"assets/{world}",
        texturedir=f"assets/{world}",
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
    collisions = add_collision_geoms(worldbody)
    sites = add_sites(worldbody)
    ET.SubElement(root, "include", file="../robots/husky.xml")
    ET.SubElement(root, "include", file="../robots/x2.xml")
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(world_path, encoding="unicode", xml_declaration=False)
    return {
        "world": world,
        "world_file": str(world_path.relative_to(REPO)),
        "visual_meshes": visual_meshes,
        "visual_textures": sorted(visual_textures),
        "ground_texture": ground_texture,
        "site_dimensions_m": site_dimensions,
        "collision_proxies": collisions,
        "sites": sites,
        "visual_triangles": sum(len(chunk["corners"]) for chunk in chunks.values()),
    }


def main():
    args = arguments()
    if not bpy.data.filepath:
        raise SystemExit("save the BlenderMCP scene before exporting it")
    if safe_name(args.world) != args.world:
        raise SystemExit("--world must contain only lowercase letters, numbers, and underscores")
    source_blend = Path(bpy.data.filepath).resolve()

    WORLD_DIR.mkdir(parents=True, exist_ok=True)
    assets_dir = WORLD_DIR / "assets" / args.world
    assets_dir.mkdir(parents=True, exist_ok=True)
    world_path = WORLD_DIR / f"{args.world}.xml"
    save_blend = (
        Path(args.save_blend).resolve() if args.save_blend
        else Path(bpy.data.filepath).resolve().with_name("simulation.blend")
    )

    link_visual_objects()
    created = create_initial_proxies()
    create_sites()
    bpy.ops.wm.save_as_mainfile(filepath=str(save_blend))
    manifest = build_mjcf(args.world, assets_dir, world_path)
    manifest["source_blend"] = portable_path(source_blend)
    manifest["waypoint_mode"] = "world" if args.world_waypoints else "none"
    manifest["ground_waypoint_config"] = write_ground_waypoint_config(
        args.world, manifest["sites"], args.world_waypoints)
    air_waypoint_config, air_route = write_air_waypoint_config(
        args.world, manifest["sites"], manifest["collision_proxies"],
        manifest["site_dimensions_m"], args.world_waypoints)
    manifest["air_waypoint_config"] = air_waypoint_config
    manifest["air_route"] = air_route
    manifest["prepared_blend"] = portable_path(save_blend)
    manifest["created_initial_proxies"] = created
    manifest_path = assets_dir / "export_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")

    print(f"[blender-export] prepared: {save_blend}")
    print(f"[blender-export] world: {world_path}")
    print(
        f"[blender-export] {len(manifest['visual_meshes'])} visual meshes, "
        f"{manifest['visual_triangles']} triangles, "
        f"{len(manifest['collision_proxies'])} collision proxies")
    if manifest["ground_waypoint_config"]:
        print(
            f"[blender-export] ground waypoints: "
            f"{REPO / manifest['ground_waypoint_config']}")
        print(
            f"[blender-export] air waypoints: "
            f"{REPO / manifest['air_waypoint_config']}")
        print(
            f"[blender-export] launch: ros2 launch spar_ground "
            f"autonomy.launch.py world:={args.world}")
        print(
            f"[blender-export] launch: ros2 launch spar_air "
            f"air.launch.py world:={args.world}")


if __name__ == "__main__":
    main()
