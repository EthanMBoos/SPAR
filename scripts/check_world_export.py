#!/usr/bin/env python3
"""Compile a Blender world export and report objective scene statistics."""

import argparse
import json
import math
from pathlib import Path
import sys

import mujoco
import yaml


REPO = Path(__file__).resolve().parents[1]
SITE_SIZE_M = 40.0
SITE_SIZE_TOLERANCE_M = 0.01


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("world", help="world name or path to a world XML file")
    args = parser.parse_args()
    supplied = Path(args.world)
    world_path = (
        supplied if supplied.suffix == ".xml"
        else REPO / "sim" / "worlds" / f"{args.world}.xml"
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
    for proxy in manifest.get("collision_proxies", []):
        if proxy.get("shape") not in {"box", "cylinder"}:
            failures.append(f"unsupported proxy shape: {proxy}")
        numbers = [*proxy.get("position", []), *proxy.get("size", [])]
        if not numbers or not all(math.isfinite(value) for value in numbers):
            failures.append(f"invalid proxy transform: {proxy.get('name')}")
        if any(value <= 0 for value in proxy.get("size", [])):
            failures.append(f"non-positive proxy size: {proxy.get('name')}")
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
    print(f"[world-export] OK: {world_path}")
    print(
        f"[world-export] {model.ngeom} geoms, {model.nmesh} meshes, "
        f"{model.ntex} textures, {model.nbody} bodies")
    print(
        f"[world-export] {manifest.get('visual_triangles', 0)} visual triangles, "
        f"{len(manifest.get('collision_proxies', []))} collision proxies")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
