#!/usr/bin/env python3
"""Validate one ground-only Blender world export and its Nav2 goal contract."""

import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import yaml

try:
    from worldfile.georeference import valid_georeference
except ModuleNotFoundError:
    from georeference import valid_georeference


REPO = Path(__file__).resolve().parents[1]
SITE_SIZE_M = 40.0
SITE_SIZE_TOLERANCE_M = 0.01
HUSKY_INITIAL_ROOT_HEIGHT_M = 0.13228
HUSKY_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
SPAWN_CLEARANCE_M = 0.8
GOAL_YAW_SAMPLES = 32


def numeric_lists_close(actual, expected, tolerance=1e-6):
    return (
        isinstance(actual, list)
        and isinstance(expected, list)
        and len(actual) == len(expected)
        and all(
            isinstance(left, (int, float))
            and isinstance(right, (int, float))
            and math.isclose(left, right, abs_tol=tolerance)
            for left, right in zip(actual, expected)
        )
    )


def manifest_datum(manifest):
    reference = manifest.get("georeference", {})
    return [
        reference.get("latitude_deg"),
        reference.get("longitude_deg"),
        reference.get("altitude_m"),
    ]


def inspect_obj_assets(assets_dir, filenames, prefix, failures):
    totals = {"vertices": 0, "faces": 0}
    for filename in filenames:
        if not filename.startswith(prefix) or not filename.endswith(".obj"):
            failures.append(f"invalid generated OBJ name: {filename}")
            continue
        path = assets_dir / filename
        if not path.is_file():
            failures.append(f"missing generated OBJ: {filename}")
            continue
        with path.open(encoding="utf-8") as stream:
            for line in stream:
                if line.startswith("v "):
                    totals["vertices"] += 1
                elif line.startswith("f "):
                    totals["faces"] += 1
    return totals


def inspect_mjcf_structure(world_path, manifest, failures):
    try:
        root = ET.parse(world_path).getroot()
    except ET.ParseError as exc:
        failures.append(f"MJCF XML is invalid: {exc}")
        return

    compiler = root.find("compiler")
    expected_assetdir = f"assets/{world_path.stem}"
    if compiler is None or compiler.get("assetdir") != expected_assetdir:
        failures.append("MJCF compiler must use the world assetdir")
    model_assets = {model.get("name") for model in root.findall("./asset/model")}
    if model_assets != {"husky_model"}:
        failures.append("MJCF must declare only the Husky model asset")
    if root.findall("include"):
        failures.append("the Husky must be attached at its authored spawn")

    worldbody = root.find("worldbody")
    if worldbody is None:
        failures.append("MJCF has no worldbody")
        return
    attachments = {
        attach.get("body") for attach in worldbody.findall("./frame/attach")
    }
    if attachments != {"base_link"}:
        failures.append("MJCF must attach only the Husky root")
    floor = worldbody.find("geom[@name='floor']")
    if floor is None or floor.get("class") != "worldfile_world_ground":
        failures.append("floor does not use worldfile_world_ground")
    if worldbody.find("geom[@name='dock_pad']") is None:
        failures.append("MJCF has no ground dock pad")

    expected_sites = {
        site["name"].lower() for site in manifest.get("sites", [])
    }
    actual_sites = {
        site.get("name") for site in worldbody.findall("site")
    }
    if expected_sites != actual_sites:
        failures.append("MJCF sites differ from the export manifest")


def inspect_goal_contacts(model, data, goal_sites, failures):
    base_joint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    footprint = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_GEOM, "husky_kinematic_footprint")
    if base_joint < 0 or footprint < 0:
        return
    base_qpos = int(model.jnt_qposadr[base_joint])
    initial_qpos = data.qpos.copy()
    root_height = float(initial_qpos[base_qpos + 2])
    for site in goal_sites:
        obstructions = set()
        yaws = [float(site["yaw"])] + [
            2.0 * math.pi * index / GOAL_YAW_SAMPLES
            for index in range(GOAL_YAW_SAMPLES)
        ]
        for yaw in yaws:
            data.qpos[:] = initial_qpos
            data.qpos[base_qpos:base_qpos + 7] = (
                site["position"][0],
                site["position"][1],
                root_height,
                math.cos(yaw / 2.0),
                0.0,
                0.0,
                math.sin(yaw / 2.0),
            )
            mujoco.mj_forward(model, data)
            for contact in data.contact:
                if contact.geom1 != footprint and contact.geom2 != footprint:
                    continue
                other = contact.geom2 if contact.geom1 == footprint else contact.geom1
                name = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_GEOM, other)
                obstructions.add(name or f"geom {other}")
        if obstructions:
            failures.append(
                f"{site['name']} obstructs the Husky turning footprint with "
                + ", ".join(sorted(obstructions))
            )
    data.qpos[:] = initial_qpos
    mujoco.mj_forward(model, data)


def validate_sites_and_config(manifest, assets_dir, failures):
    sites = manifest.get("sites", [])
    for site in sites:
        numbers = [*site.get("position", []), site.get("yaw")]
        if not all(
            isinstance(value, (int, float)) and math.isfinite(value)
            for value in numbers
        ):
            failures.append(f"invalid site transform: {site.get('name')}")
    spawn_sites = [site for site in sites if site.get("role") == "husky_spawn"]
    dock_sites = [site for site in sites if site.get("role") == "dock"]
    goal_sites = [
        site for site in sites
        if site.get("role", "").startswith("navigation_goal_")
    ]
    if len(spawn_sites) != 1:
        failures.append("world needs exactly one husky_spawn site")
    if len(dock_sites) != 1:
        failures.append("world needs exactly one dock site")
    if len(goal_sites) < 3:
        failures.append("world needs at least three navigation goals")
    if len(spawn_sites) + len(dock_sites) + len(goal_sites) != len(sites):
        failures.append("world contains unsupported semantic sites")

    orders = [site.get("order") for site in goal_sites]
    if (
        any(not isinstance(order, int) or isinstance(order, bool) for order in orders)
        or len(set(orders)) != len(orders)
    ):
        failures.append("navigation goals need unique integer orders")
    goal_sites.sort(key=lambda site: site.get("order", -1))

    config = manifest.get("navigation_config")
    config_path = REPO / config if isinstance(config, str) else None
    if config_path is None or not config_path.is_file():
        failures.append("navigation config is missing")
        return sites, spawn_sites, goal_sites
    try:
        document = yaml.safe_load(config_path.read_text())
        datum = document["navsat_datum"]
        dock = document["dock_pose"]
        configured_goals = document["navigation_goals"]
        expected_datum = manifest_datum(manifest)
        if (
            not numeric_lists_close(datum[:2], expected_datum[:2])
            or type(datum[2]) is not float
            or not math.isclose(datum[2], 0.0, abs_tol=1e-9)
        ):
            failures.append("navigation datum differs from the world")
        if (
            not isinstance(dock, list)
            or len(dock) != 3
            or not all(
                type(value) is float and math.isfinite(value)
                for value in dock
            )
        ):
            failures.append(
                "navigation config dock_pose must contain three finite floats"
            )
        elif dock_sites:
            expected_dock = [
                dock_sites[0]["position"][0],
                dock_sites[0]["position"][1],
                math.atan2(
                    math.sin(dock_sites[0]["yaw"]),
                    math.cos(dock_sites[0]["yaw"]),
                ),
            ]
            if not numeric_lists_close(dock, expected_dock):
                failures.append(
                    "navigation config dock pose differs from authored site")
        if len(configured_goals) != len(goal_sites):
            failures.append("navigation config goal count differs from authored sites")
        for configured, site in zip(configured_goals, goal_sites):
            expected = [
                site["position"][0],
                site["position"][1],
                math.atan2(math.sin(site["yaw"]), math.cos(site["yaw"])),
            ]
            actual = [configured.get(key) for key in ("x", "y", "yaw")]
            if configured.get("name") != site["name"].lower():
                failures.append("navigation config goal name differs from authored site")
            if not numeric_lists_close(actual, expected):
                failures.append("navigation config goal pose differs from authored site")
    except (KeyError, TypeError, yaml.YAMLError) as exc:
        failures.append(f"invalid navigation config: {exc}")

    contract = manifest.get("navigation_goals") or {}
    if contract.get("frame") != "map_world_enu":
        failures.append("navigation goals are not in the world-aligned map frame")
    if len(contract.get("goals", [])) != len(goal_sites):
        failures.append("navigation goal manifest differs from authored sites")
    if contract.get("goal_footprints_clear") is not True:
        failures.append("navigation goal footprint clearance was not recorded")
    observation = contract.get("first_observation") or {}
    if (
        observation.get("order") != 0
        or not isinstance(observation.get("range_m"), (int, float))
        or observation.get("range_m", math.inf)
        > contract.get("detector_range_limit_m", -math.inf)
        or observation.get("route_travel_m", math.inf)
        > contract.get("observation_travel_limit_m", -math.inf)
        or observation.get("heading_error_rad", math.inf)
        > contract.get("heading_tolerance_rad", -math.inf)
    ):
        failures.append(
            "first navigation goal does not guarantee early red-target perception")
    if contract.get("line_of_sight_checked_in_blender") is not True:
        failures.append("red-target line-of-sight check was not recorded")
    return sites, spawn_sites, goal_sites


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
    if not valid_georeference(manifest.get("georeference")):
        failures.append("manifest has no valid world georeference")
    dimensions = manifest.get("site_dimensions_m", [])
    if not numeric_lists_close(dimensions, [SITE_SIZE_M, SITE_SIZE_M], SITE_SIZE_TOLERANCE_M):
        failures.append("world footprint is not exactly 40 by 40 metres")

    colliders = manifest.get("colliders", [])
    for collider in colliders:
        numbers = [*collider.get("position", []), *collider.get("size", [])]
        if not numbers or not all(math.isfinite(value) for value in numbers):
            failures.append(f"invalid collider transform: {collider.get('name')}")
    target_roots = {
        collider.get("source_root") for collider in colliders
        if collider.get("source_role") == "anomaly"
        or collider.get("source_type") == "inspection_target"
    }
    if len(target_roots) != 1:
        failures.append("world needs exactly one red inspection target root")
    sites, spawn_sites, goal_sites = validate_sites_and_config(
        manifest, assets_dir, failures)

    visual_counts = inspect_obj_assets(
        assets_dir, manifest.get("visual_meshes", []), "mesh_", failures)
    inspect_obj_assets(
        assets_dir, manifest.get("collision_meshes", []), "colmesh_", failures)
    if visual_counts["faces"] != manifest.get("visual_triangles"):
        failures.append("visual OBJ triangle count differs from manifest")
    inspect_mjcf_structure(world_path, manifest, failures)

    try:
        model = mujoco.MjModel.from_xml_path(str(world_path))
    except Exception as exc:
        failures.append(f"MuJoCo could not compile the world: {exc}")
        model = None
    if model is not None:
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        numeric_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_NUMERIC, "worldfile.world_georeference")
        if numeric_id < 0:
            failures.append("compiled world has no geographic reference")
        else:
            address = int(model.numeric_adr[numeric_id])
            size = int(model.numeric_size[numeric_id])
            actual = model.numeric_data[address:address + size].tolist()
            if not numeric_lists_close(actual, manifest_datum(manifest)):
                failures.append("compiled geographic reference differs from manifest")

        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if body < 0 or not spawn_sites:
            failures.append("compiled world cannot verify the Husky spawn")
        else:
            expected = np.array(spawn_sites[0]["position"], dtype=float)
            expected[2] += HUSKY_INITIAL_ROOT_HEIGHT_M
            if not np.allclose(data.xpos[body], expected, atol=1e-6):
                failures.append("compiled Husky position differs from its spawn")
        for wheel in HUSKY_WHEELS:
            joint = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, f"{wheel}_wheel_joint")
            actuator = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{wheel}_wheel_actuator")
            if joint < 0 or actuator >= 0:
                failures.append(f"invalid kinematic wheel contract: {wheel}")
        inspect_goal_contacts(model, data, goal_sites, failures)

        passive = mujoco.MjData(model)
        for _ in range(round(5.0 / model.opt.timestep)):
            mujoco.mj_step(model, passive)
        if not np.isfinite(passive.qpos).all() or not np.isfinite(passive.qvel).all():
            failures.append("five-second passive rollout produced non-finite state")
        if any(int(warning.number) for warning in passive.warning):
            failures.append("five-second passive rollout produced MuJoCo warnings")

    if failures:
        for failure in failures:
            print(f"[world-export] FAIL: {failure}")
        return 1
    print(
        f"[world-export] OK: {world_path.name}; "
        f"{len(manifest.get('visual_meshes', []))} visual meshes, "
        f"{len(colliders)} colliders, {len(goal_sites)} navigation goals"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
