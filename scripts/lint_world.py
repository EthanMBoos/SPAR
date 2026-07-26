#!/usr/bin/env python3
"""Validate a world MJCF against one robot before it ships.

Checks:
  1. stealth geoms  — every static collision geom must cross the robot's 2D
                      scan plane, or the robot can hit what it cannot see.
  2. dock zone      — the dock area must be clear of collision geometry, or
                      docking/recharge fails in ways that look like bugs.
  3. waypoints      — every route waypoint needs standoff from obstacles, or
                      Nav2 oscillates trying to reach an unreachable goal.
  4. geom budget    — a soft warning when the world grows past what the
                      render/raycast budget was sized for.
  5. support        — every static solid must rest on something: not buried
                      in a neighbor, not hovering above its support.
  6. settle         — step 2 s with no controller; a well-authored world is
                      at rest, a floating or leaning free prop is not.

Usage: lint_world.py --robot <name> <world.xml> [autonomy_<world>.yaml]
Exits nonzero on any hard failure.
"""
import argparse
import math
import os
import sys

import mujoco
import numpy as np
import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "sim"))
from spar_sim.robot_config import scan_site  # noqa: E402

DOCK_CLEAR_M = 1.2    # radius around the dock that must be collision-free
WAYPOINT_CLEAR_M = 0.6  # robot half-width + margin
GEOM_BUDGET = 300
REST_TOL_M = 1e-3     # resting contact tolerance for the support check
SETTLE_SEC = 2.0
SETTLE_MOVE_M = 0.15  # a settling robot drops millimeters; a falling or
                      # toppling prop moves decimeters
SETTLE_VEL = 0.1      # max dof speed once the settle window ends


def static_collision_geoms(model):
    """Static, collidable world geoms; robots and movers are excluded.

    MuJoCo weld membership handles wrapped static assets correctly:
    body_weldid == 0 is world geometry, while a joint starts a new weld group.
    """
    ids = []
    for gid in range(model.ngeom):
        if model.body_weldid[model.geom_bodyid[gid]] != 0:
            continue
        if model.geom_contype[gid] == 0 and model.geom_conaffinity[gid] == 0:
            continue
        if model.geom_type[gid] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        ids.append(gid)
    return ids


def geom_z_extent(model, data, gid):
    """World-frame z half-extent of a primitive geom (exact for the primitive
    set the authoring discipline allows; bounding sphere for anything else)."""
    size = model.geom_size[gid]
    t = model.geom_type[gid]
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return size[0]
    R = data.geom_xmat[gid].reshape(3, 3)
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return sum(abs(R[2][i]) * size[i] for i in range(3))
    if t == mujoco.mjtGeom.mjGEOM_CYLINDER:
        return abs(R[2][2]) * size[1] + math.hypot(R[2][0], R[2][1]) * size[0]
    # capsule / other: bounding sphere (conservative; fine for a gate)
    return model.geom_rbound[gid]


def geom_xy_clearance(model, data, gid, px, py):
    """Horizontal distance from a point to the geom's world-axis-aligned box,
    0 when the point is inside it.

    Not geom_rbound: that is a bounding SPHERE, tight only for compact geoms.
    A 10m wall slab has an rbound of 5.2m, so subtracting it puts the dock
    "inside" a wall that is really 4.9m away, and every long thin object a
    site is made of (fences, containers, scaffolding) fails the same way.
    This avoids the loose bounding sphere used by long thin objects."""
    rot = data.geom_xmat[gid].reshape(3, 3)
    center = data.geom_xpos[gid] + rot @ model.geom_aabb[gid, :3]
    half = np.abs(rot) @ model.geom_aabb[gid, 3:]
    dx = max(abs(px - center[0]) - half[0], 0.0)
    dy = max(abs(py - center[1]) - half[1], 0.0)
    return math.hypot(dx, dy)


def load_autonomy_yaml(path):
    """Pull dock pose and patrol waypoints out of the ros params yaml."""
    with open(path) as f:
        doc = yaml.safe_load(f)
    merged = {}
    for node in doc.values():  # {node_name: {ros__parameters: {...}}}
        if isinstance(node, dict):
            merged.update(node.get("ros__parameters", {}))
    dock = (merged.get("dock_x"), merged.get("dock_y"))
    wx = merged.get("waypoints_x", [])
    wy = merged.get("waypoints_y", [])
    return {
        "dock": dock if dock[0] is not None else None,
        "waypoints": list(zip(wx, wy)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--robot", required=True,
                        help="robot custom-config prefix, e.g. husky")
    parser.add_argument("model", help="MJCF world file")
    parser.add_argument("autonomy", nargs="?",
                        help="optional autonomy_<world>.yaml")
    args = parser.parse_args()
    model_path = args.model
    cfg_path = args.autonomy

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    geoms = static_collision_geoms(model)
    try:
        scan_name, scan_id = scan_site(model, args.robot)
    except ValueError as exc:
        parser.error(str(exc))
    z_lidar = float(data.site_xpos[scan_id][2])
    failures, warnings = [], []

    def gname(gid):
        return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom#{gid}"

    # 1. stealth geoms: static solids must cross the robot's scan plane.
    for gid in geoms:
        z = data.geom_xpos[gid][2]
        half = geom_z_extent(model, data, gid)
        top, bottom = z + half, z - half
        if top < z_lidar:
            failures.append(
                f"stealth obstacle: '{gname(gid)}' tops out at {top:.2f}m, "
                f"below robot '{args.robot}' scan site '{scan_name}' at "
                f"{z_lidar:.2f}m — robot can hit what it can't see")
        elif bottom > z_lidar:
            warnings.append(
                f"overhang: '{gname(gid)}' starts at {bottom:.2f}m, above the "
                f"'{scan_name}' plane — invisible to this robot's 2D scan "
                "(ok if intended)")

    cfg = load_autonomy_yaml(cfg_path) if cfg_path else None

    # 2. dock zone clear.
    if cfg and cfg["dock"]:
        dx, dy = cfg["dock"]
        for gid in geoms:
            if geom_xy_clearance(model, data, gid, dx, dy) < DOCK_CLEAR_M:
                failures.append(
                    f"dock zone: '{gname(gid)}' intrudes within {DOCK_CLEAR_M}m "
                    f"of the dock ({dx}, {dy})")

    # 3. waypoint standoff.
    if cfg:
        for i, (wx, wy) in enumerate(cfg["waypoints"]):
            for gid in geoms:
                clear = geom_xy_clearance(model, data, gid, wx, wy)
                if clear < WAYPOINT_CLEAR_M:
                    failures.append(
                        f"waypoint {i} ({wx}, {wy}): only {clear:.2f}m from "
                        f"'{gname(gid)}' (needs {WAYPOINT_CLEAR_M}m)")

    # 4. budget.
    if model.ngeom > GEOM_BUDGET:
        warnings.append(f"{model.ngeom} geoms exceeds the {GEOM_BUDGET} budget "
                        f"the render/raycast loop was sized for")

    # 5. support: every static solid rests on the floor or a neighbor.
    # All worldbody geoms live in body 0, so the contact machinery never
    # reports static-static pairs (same-body exclusion); mj_geomDistance
    # asks the narrowphase directly and returns signed distance, negative
    # when penetrating.
    floor_planes = [
        g for g in range(model.ngeom)
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE
        and (model.geom_contype[g] or model.geom_conaffinity[g])
        and model.body_weldid[model.geom_bodyid[g]] == 0]
    for gid in geoms:
        dmin = math.inf
        for other in list(geoms) + floor_planes:
            if other != gid:
                dmin = min(dmin, mujoco.mj_geomDistance(
                    model, data, gid, other, 1.0, None))
        if dmin < -REST_TOL_M:
            failures.append(
                f"interpenetration: '{gname(gid)}' overlaps a neighbor "
                f"by {-dmin:.3f}m")
        elif dmin > REST_TOL_M:
            gap = f"{dmin:.3f}m above" if dmin < 1.0 else "over 1m from"
            failures.append(
                f"floating: '{gname(gid)}' hangs {gap} its nearest support")

    # 6. settle: a world at rest stays at rest. Free props authored
    # floating fall, leaning ones topple, and a robot spawned intersecting
    # anything gets ejected; all of that is large motion with no controller.
    # Runs last: it advances the physics the earlier checks read at t=0.
    start = data.xpos.copy()
    for _ in range(int(SETTLE_SEC / model.opt.timestep)):
        mujoco.mj_step(model, data)
    for b in range(1, model.nbody):
        moved = math.dist(data.xpos[b], start[b])
        if moved > SETTLE_MOVE_M:
            bname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body#{b}"
            failures.append(
                f"settle: body '{bname}' moved {moved:.2f}m in the first "
                f"{SETTLE_SEC:.0f}s with no controller")
    vmax = max((abs(float(v)) for v in data.qvel), default=0.0)
    if vmax > SETTLE_VEL:
        failures.append(
            f"settle: still moving after {SETTLE_SEC:.0f}s "
            f"(max dof speed {vmax:.2f})")

    for w in warnings:
        print(f"[lint] WARN: {w}")
    for f_ in failures:
        print(f"[lint] FAIL: {f_}")
    if failures:
        print(f"[lint] {len(failures)} failure(s) in {os.path.basename(model_path)}")
        return 1
    print(f"[lint] OK: {os.path.basename(model_path)} "
          f"({len(geoms)} static solids, {model.ngeom} geoms total)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
