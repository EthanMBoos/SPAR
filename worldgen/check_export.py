#!/usr/bin/env python3
"""Compile a Blender world export and report objective scene statistics."""

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml


REPO = Path(__file__).resolve().parents[1]
SITE_SIZE_M = 40.0
SITE_SIZE_TOLERANCE_M = 0.01
EXPECTED_CONTACT_FRICTION = {
    "caster_front": 0.005,
    "caster_rear": 0.005,
    "drive_wheel_left": 0.9,
    "drive_wheel_right": 0.9,
}


def inspect_obj_assets(assets_dir, filenames, failures):
    totals = {"vertices": 0, "normals": 0, "faces": 0}
    for filename in filenames:
        if not filename.startswith("mesh_") or not filename.endswith(".obj"):
            failures.append(f"visual mesh does not use mesh_* naming: {filename}")
            continue
        path = assets_dir / filename
        if not path.is_file():
            continue
        counts = {"vertices": 0, "normals": 0, "faces": 0}
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("v "):
                    counts["vertices"] += 1
                elif line.startswith("vn "):
                    counts["normals"] += 1
                elif line.startswith("f "):
                    counts["faces"] += 1
                    if any(token.count("/") != 2 for token in line.split()[1:]):
                        failures.append(f"OBJ face lacks position/UV/normal indices: {filename}")
                        break
        if counts["faces"] and not counts["normals"]:
            failures.append(f"OBJ has no authored normals: {filename}")
        for key, value in counts.items():
            totals[key] += value
    if totals["faces"] and totals["vertices"] >= totals["faces"] * 3:
        failures.append("visual OBJ meshes do not share indexed vertices")
    return totals


def inspect_collision_obj_assets(assets_dir, filenames, failures):
    totals = {"vertices": 0, "faces": 0}
    for filename in filenames:
        if not filename.startswith("colmesh_") or not filename.endswith(".obj"):
            failures.append(f"collision mesh does not use colmesh_* naming: {filename}")
            continue
        path = assets_dir / filename
        if not path.is_file():
            failures.append(f"missing collision mesh: {filename}")
            continue
        vertices = 0
        faces = 0
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("v "):
                    vertices += 1
                elif line.startswith("f "):
                    faces += 1
                    if any("/" in token for token in line.split()[1:]):
                        failures.append(
                            f"collision OBJ unnecessarily contains split indices: {filename}")
                        break
        if vertices < 4 or faces < 4:
            failures.append(f"collision OBJ is not volumetric: {filename}")
        totals["vertices"] += vertices
        totals["faces"] += faces
    return totals


def inspect_mjcf_structure(world_path, manifest, failures):
    try:
        root = ET.parse(world_path).getroot()
    except (OSError, ET.ParseError) as exc:
        failures.append(f"invalid MJCF XML: {exc}")
        return

    compiler = root.find("compiler")
    expected_assetdir = f"assets/{world_path.stem}"
    if compiler is None or compiler.get("assetdir") != expected_assetdir:
        failures.append("MJCF compiler must use the world assetdir")
    elif compiler.get("meshdir") or compiler.get("texturedir"):
        failures.append("MJCF compiler redundantly sets meshdir or texturedir")

    classes = {
        element.get("class")
        for element in root.findall("./default/default")
    }
    required_classes = {
        "spar_world_visual",
        "spar_world_collision",
        "spar_world_ground",
        "spar_world_site",
    }
    missing_classes = required_classes - classes
    if missing_classes:
        failures.append(
            "MJCF is missing defaults classes: " + ", ".join(sorted(missing_classes)))

    for mesh in root.findall("./asset/mesh"):
        if mesh.get("name", "").startswith("mesh_"):
            if mesh.get("inertia") or mesh.get("smoothnormal"):
                failures.append(
                    f"visual mesh has redundant inferred properties: {mesh.get('name')}")

    worldbody = root.find("worldbody")
    if worldbody is None:
        failures.append("MJCF has no worldbody")
        return
    floor = worldbody.find("geom[@name='floor']")
    if floor is None or floor.get("class") != "spar_world_ground":
        failures.append("floor does not use spar_world_ground")
    for geom in worldbody.findall("geom"):
        name = geom.get("name", "")
        if name.startswith("vis_") and geom.get("class") != "spar_world_visual":
            failures.append(f"visual geom has wrong defaults class: {name}")
        if name.startswith("col_"):
            if not name.startswith("col_auto_"):
                failures.append(f"world contains a non-v1 collider geom: {name}")
            if geom.get("class") != "spar_world_collision":
                failures.append(f"collision geom has wrong defaults class: {name}")
            if geom.get("rgba") is not None or geom.get("group") is not None:
                failures.append(f"collision geom overrides shared visibility: {name}")
            mesh_name = geom.get("mesh", "")
            if geom.get("type") != "mesh" or not mesh_name.startswith("colmesh_"):
                failures.append(
                    f"automatic collider does not use a per-element mesh: {name}")
    for site in worldbody.findall("site"):
        if site.get("name", "").startswith("site_"):
            if site.get("class") != "spar_world_site":
                failures.append(f"site has wrong defaults class: {site.get('name')}")


def compiled_contact_friction(model, data, geom_name):
    for contact in data.contact:
        names = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1),
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2),
        }
        if names == {"floor", geom_name}:
            return float(contact.friction[0])
    return None


def warning_counts(data):
    return [int(warning.number) for warning in data.warning]


def controlled_husky_rollout(model, timestep):
    original_timestep = model.opt.timestep
    model.opt.timestep = timestep
    try:
        data = mujoco.MjData(model)
        body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        left_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_left")
        right_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_right")
        if min(body_id, left_id, right_id) < 0:
            raise RuntimeError("compiled world is missing the Husky rollout contract")
        for _ in range(round(1.0 / timestep)):
            data.ctrl[left_id] = 3.0
            data.ctrl[right_id] = 3.0
            mujoco.mj_step(model, data)
        forward_position = data.xpos[body_id].copy()
        for _ in range(round(1.0 / timestep)):
            data.ctrl[left_id] = 2.0
            data.ctrl[right_id] = -2.0
            mujoco.mj_step(model, data)
        return forward_position, data.xpos[body_id].copy(), warning_counts(data)
    finally:
        model.opt.timestep = original_timestep


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", help="world name or path to a world XML file")
    args = parser.parse_args()
    supplied = Path(args.world)
    world_path = (
        supplied if supplied.suffix == ".xml"
        else REPO / "sim" / "worlds" / f"{args.world.lower()}.xml"
    ).resolve()
    if not world_path.is_file():
        sys.exit(f"world does not exist: {world_path}")

    assets_dir = world_path.parent / "assets" / world_path.stem
    manifest_path = assets_dir / "export_manifest.json"
    if not manifest_path.is_file():
        sys.exit(f"export manifest does not exist: {manifest_path}")
    try:
        manifest = json.loads(manifest_path.read_text())
    except (OSError, json.JSONDecodeError) as exc:
        sys.exit(f"invalid export manifest: {exc}")

    failures = []
    if manifest.get("world") != world_path.stem:
        failures.append("manifest world does not match the MJCF filename")
    generation_recipe = manifest.get("generation")
    generation_hash = manifest.get("generation_sha256")
    if generation_recipe is not None or generation_hash is not None:
        if not isinstance(generation_recipe, dict):
            failures.append("embedded generation recipe is invalid")
        else:
            if generation_recipe.get("world") != world_path.stem:
                failures.append("generation recipe world does not match export")
            encoded_recipe = (
                json.dumps(generation_recipe, indent=2) + "\n"
            ).encode()
            if hashlib.sha256(encoded_recipe).hexdigest() != generation_hash:
                failures.append("generation recipe hash does not match export")
    dimensions = manifest.get("site_dimensions_m", [])
    if (
        len(dimensions) != 2
        or not all(isinstance(value, (int, float)) for value in dimensions)
        or not all(math.isfinite(value) for value in dimensions)
        or any(
            not math.isclose(value, SITE_SIZE_M, abs_tol=SITE_SIZE_TOLERANCE_M)
            for value in dimensions
        )
    ):
        failures.append("generated world dimensions must be 40 by 40 m")

    referenced_assets = set(manifest.get("visual_meshes", []))
    referenced_assets.update(manifest.get("collision_meshes", []))
    referenced_assets.update(manifest.get("visual_textures", []))
    if manifest.get("ground_texture"):
        referenced_assets.add(manifest["ground_texture"])
    for filename in manifest.get("visual_meshes", []):
        if not (assets_dir / filename).is_file():
            failures.append(f"missing visual mesh: {filename}")
    for filename in manifest.get("visual_textures", []):
        if not (assets_dir / filename).is_file():
            failures.append(f"missing visual texture: {filename}")
    ground_texture = manifest.get("ground_texture")
    if ground_texture and not (assets_dir / ground_texture).is_file():
        failures.append(f"missing ground texture: {ground_texture}")
    actual_assets = {
        path.name for path in assets_dir.iterdir()
        if path.is_file() and path.name != manifest_path.name
    }
    unexpected_assets = actual_assets - referenced_assets
    if unexpected_assets:
        failures.append(
            "unreferenced generated assets: "
            + ", ".join(sorted(unexpected_assets)))
    colliders = manifest.get("colliders", [])
    if not colliders:
        failures.append("export contains no automatic colliders")
    if len(manifest.get("collision_meshes", [])) != len(colliders):
        failures.append("collider count does not match collision mesh assets")
    for collider in colliders:
        if collider.get("mesh") not in manifest.get("collision_meshes", []):
            failures.append(
                f"collider has no referenced mesh: {collider.get('name')}")
        numbers = [*collider.get("position", []), *collider.get("size", [])]
        if not numbers or not all(math.isfinite(value) for value in numbers):
            failures.append(f"invalid collider transform: {collider.get('name')}")
        if any(value <= 0 for value in collider.get("size", [])):
            failures.append(f"non-positive collider size: {collider.get('name')}")
        if not collider.get("name") or not collider.get("source_type"):
            failures.append(f"collider has no semantic source: {collider.get('name')}")
    for omission in manifest.get("collision_omissions", []):
        if not omission.get("source_root") or not omission.get("reason"):
            failures.append(f"invalid collision omission: {omission}")
    for opt_out in manifest.get("collision_opt_outs", []):
        if (
            not opt_out.get("name")
            or not opt_out.get("source_root")
            or opt_out.get("reason") != "spar_no_collision"
        ):
            failures.append(f"invalid collision opt-out: {opt_out}")
    sites = manifest.get("sites", [])
    for site in sites:
        numbers = [*site.get("position", []), site.get("yaw")]
        if not all(isinstance(value, (int, float)) and math.isfinite(value)
                   for value in numbers):
            failures.append(f"invalid site transform: {site.get('name')}")
    roles = {site.get("role") for site in sites}
    required_roles = {"husky_spawn", "x2_spawn", "dock"}
    if manifest.get("waypoint_mode") == "world":
        ground_sites = [site for site in sites
                        if site.get("role", "").startswith("patrol_")]
        if len(ground_sites) < 3:
            failures.append("world waypoint mode needs at least three patrol sites")
        orders = [site.get("patrol_order") for site in ground_sites]
        if None in orders or len(set(orders)) != len(orders):
            failures.append("world patrol sites need unique spar_patrol_order values")
        ground_config = manifest.get("ground_waypoint_config")
        ground_path = REPO / ground_config if ground_config else None
        if ground_path is None or not ground_path.is_file():
            failures.append("ground waypoint file is missing")
        else:
            try:
                document = yaml.safe_load(ground_path.read_text())
                waypoints = document["/**/bt_executive"]["ros__parameters"][
                    "patrol_waypoints"]
                if len(waypoints) != len(ground_sites) * 3:
                    failures.append(
                        "ground waypoint count does not match patrol sites")
                if not all(type(value) is float for value in waypoints):
                    failures.append("ground waypoints must all be ROS doubles")
                elif not all(math.isfinite(value) for value in waypoints):
                    failures.append("ground waypoints must all be finite")
            except (KeyError, TypeError, yaml.YAMLError) as exc:
                failures.append(f"invalid ground waypoint file: {exc}")

        ground_route = manifest.get("ground_route") or {}
        ground_manifest_waypoints = ground_route.get("waypoints", [])
        if len(ground_manifest_waypoints) != len(ground_sites):
            failures.append("ground route manifest does not match patrol sites")
        observation = ground_route.get("first_observation") or {}
        observation_range = observation.get("range_m")
        heading_error = observation.get("heading_error_rad")
        observation_travel = observation.get("route_travel_m")
        if not isinstance(observation_range, (int, float)) or observation_range > 8.0:
            failures.append("ground route has no observation within 8 m")
        if (
            not isinstance(heading_error, (int, float))
            or heading_error > math.radians(25.0)
        ):
            failures.append("ground route does not face the inspection target")
        if not isinstance(observation_travel, (int, float)) or observation_travel > 20.0:
            failures.append("ground route reaches the inspection view too late")
        if ground_route.get("line_of_sight_checked_in_blender") is not True:
            failures.append("ground inspection line of sight was not checked")

        air_sites = [site for site in sites
                     if site.get("role", "").startswith("air_patrol_")]
        if len(air_sites) < 3:
            failures.append("world waypoint mode needs at least three air patrol sites")
        air_orders = [site.get("patrol_order") for site in air_sites]
        if None in air_orders or len(set(air_orders)) != len(air_orders):
            failures.append("air patrol sites need unique spar_patrol_order values")
        air_config = manifest.get("air_waypoint_config")
        air_path = REPO / air_config if air_config else None
        if air_path is None or not air_path.is_file():
            failures.append("air waypoint file is missing")
        else:
            try:
                document = yaml.safe_load(air_path.read_text())
                parameters = document["/**/bt_executive"]["ros__parameters"]
                air_waypoints = parameters["patrol_waypoints"]
                if len(air_waypoints) != len(air_sites) * 4:
                    failures.append(
                        "air waypoint count does not match air patrol sites")
                if not all(type(value) is float for value in air_waypoints):
                    failures.append("air waypoints must all be ROS doubles")
                elif not all(math.isfinite(value) for value in air_waypoints):
                    failures.append("air waypoints must all be finite")
                for key in ("cruise_alt_m", "orbit_alt_m"):
                    value = parameters.get(key)
                    if type(value) is not float or not math.isfinite(value):
                        failures.append(f"air {key} must be a ROS double")
            except (KeyError, TypeError, yaml.YAMLError) as exc:
                failures.append(f"invalid air waypoint file: {exc}")

        air_route = manifest.get("air_route") or {}
        manifest_waypoints = air_route.get("waypoints", [])
        if len(manifest_waypoints) != len(air_sites):
            failures.append("air route manifest does not match air patrol sites")
        flight_floor = air_route.get("collision_clear_flight_floor_m")
        if not isinstance(flight_floor, (int, float)) or not math.isfinite(flight_floor):
            failures.append("air route has no finite collision-clear flight floor")
        elif any(waypoint.get("z", -math.inf) < flight_floor
                 for waypoint in manifest_waypoints):
            failures.append("an air waypoint is below the collision-clear flight floor")
        anomaly_range = air_route.get("closest_anomaly_range_m")
        if (
            not isinstance(anomaly_range, (int, float))
            or not math.isfinite(anomaly_range)
            or anomaly_range > 12.0
        ):
            failures.append("air route does not bring the anomaly within 12 m")
        if air_route.get("takeoff_corridor_clear") is not True:
            failures.append("air takeoff and landing corridor is not clear")
    missing_roles = required_roles - roles
    if missing_roles:
        failures.append(f"missing semantic sites: {', '.join(sorted(missing_roles))}")

    obj_counts = inspect_obj_assets(
        assets_dir, manifest.get("visual_meshes", []), failures)
    collision_obj_counts = inspect_collision_obj_assets(
        assets_dir, manifest.get("collision_meshes", []), failures)
    inspect_mjcf_structure(world_path, manifest, failures)

    if failures:
        for failure in failures:
            print(f"[world-export] FAIL: {failure}")
        return 1

    try:
        model = mujoco.MjModel.from_xml_path(str(world_path))
    except Exception as exc:
        print(f"[world-export] FAIL: MuJoCo could not compile the world: {exc}")
        return 1

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for geom_name, expected in EXPECTED_CONTACT_FRICTION.items():
        actual = compiled_contact_friction(model, data, geom_name)
        if actual is None:
            failures.append(f"initial state has no floor contact for {geom_name}")
        elif not math.isclose(actual, expected, abs_tol=1e-9):
            failures.append(
                f"{geom_name} floor friction is {actual:g}, expected {expected:g}")

    passive = mujoco.MjData(model)
    for _ in range(round(5.0 / model.opt.timestep)):
        mujoco.mj_step(model, passive)
    if not np.isfinite(passive.qpos).all() or not np.isfinite(passive.qvel).all():
        failures.append("five-second passive rollout produced non-finite state")
    if any(warning_counts(passive)):
        failures.append("five-second passive rollout produced MuJoCo warnings")

    try:
        coarse = controlled_husky_rollout(model, 0.002)
        fine = controlled_husky_rollout(model, 0.001)
    except RuntimeError as exc:
        failures.append(str(exc))
    else:
        if coarse[0][0] < 0.1:
            failures.append("Husky did not move forward under symmetric wheel control")
        position_delta = float(np.linalg.norm(coarse[1][:2] - fine[1][:2]))
        if position_delta > 0.05:
            failures.append(
                f"Husky rollout changes {position_delta:.3g} m when timestep is halved")
        if any(coarse[2]) or any(fine[2]):
            failures.append("controlled Husky rollout produced MuJoCo warnings")

    if failures:
        for failure in failures:
            print(f"[world-export] FAIL: {failure}")
        return 1

    print(f"[world-export] OK: {world_path}")
    print(
        f"[world-export] {model.ngeom} geoms, {model.nmesh} meshes, "
        f"{model.ntex} textures, {model.nbody} bodies")
    print(
        f"[world-export] {manifest.get('visual_triangles', 0)} visual triangles, "
        f"{len(colliders)} scene colliders")
    print(
        f"[world-export] indexed OBJ: {obj_counts['vertices']} vertices, "
        f"{obj_counts['normals']} normals, {obj_counts['faces']} faces")
    if collision_obj_counts["faces"]:
        print(
            f"[world-export] per-element collision OBJ: "
            f"{collision_obj_counts['vertices']} vertices, "
            f"{collision_obj_counts['faces']} faces")
    print("[world-export] passive and controlled 0.002/0.001 s rollouts passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
