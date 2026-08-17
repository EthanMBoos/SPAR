"""MuJoCo ray queries against SPAR's invisible collision geometry."""

from __future__ import annotations

import mujoco
import numpy as np


WORLD_COLLISION_GROUP = 3

# A compact vertical fan with dense coverage below the horizon, where short
# pallets and curbs appear as the Husky approaches. These are genuine rays
# from the lidar origin; their closest obstacle returns are later projected
# into the planar LaserScan consumed by Nav2.
NAV_LIDAR_ELEVATIONS_DEG = (
    -60.0,
    -45.0,
    -35.0,
    -30.0,
    -27.0,
    -24.0,
    -21.0,
    -18.0,
    -15.0,
    -12.0,
    -10.0,
    -8.0,
    -6.0,
    -4.0,
    -2.0,
    0.0,
    2.0,
    4.0,
    6.0,
    9.0,
    12.0,
    16.0,
    21.0,
    27.0,
)
NAV_LIDAR_MIN_OBSTACLE_Z_M = 0.05
NAV_LIDAR_MAX_OBSTACLE_Z_M = 0.80


def cast_world_collision_rays(
    model,
    data,
    origin,
    directions,
    geom_groups,
    body_exclude,
    geom_ids,
    distances,
    ray_count,
    cutoff,
):
    """Cast rays against collision meshes without making them renderable.

    MuJoCo's ray functions ignore geoms whose alpha is exactly zero. SPAR's
    collision meshes use alpha zero because a separate, detailed mesh owns
    appearance. Temporarily giving collision geoms nonzero alpha makes them
    available to the ray query; restoring it immediately keeps renderers and
    viewers unchanged.
    """
    collision_geoms = model.geom_group == WORLD_COLLISION_GROUP
    previous_alpha = model.geom_rgba[collision_geoms, 3].copy()
    model.geom_rgba[collision_geoms, 3] = 1.0
    try:
        mujoco.mj_multiRay(
            model,
            data,
            origin,
            directions,
            geom_groups,
            True,
            body_exclude,
            geom_ids,
            distances,
            None,
            ray_count,
            cutoff,
        )
    finally:
        model.geom_rgba[collision_geoms, 3] = previous_alpha


class NavigationRayFan:
    """Project a multi-channel collision scan into planar navigation ranges."""

    def __init__(self, bearing_count, max_horizontal_range):
        self._bearing_count = int(bearing_count)
        self._max_horizontal_range = float(max_horizontal_range)
        self._elevations = np.deg2rad(NAV_LIDAR_ELEVATIONS_DEG)
        self._cos_elevations = np.cos(self._elevations)
        self._ray_count = self._bearing_count * len(self._elevations)
        self._directions = np.empty(
            (len(self._elevations), self._bearing_count, 3), dtype=np.float64)
        self._geom_ids = np.full(self._ray_count, -1, dtype=np.int32)
        self._distances = np.full(self._ray_count, -1.0, dtype=np.float64)
        # Group 0 contains dynamic/default robot geometry; group 3 contains
        # exported world collision meshes. Ground is group 4 and is excluded.
        self._geom_groups = np.array(
            [1, 0, 0, 1, 0, 0], dtype=np.uint8)

    def scan(self, model, data, origin, world_bearings, body_exclude):
        if len(world_bearings) != self._bearing_count:
            raise ValueError(
                f"expected {self._bearing_count} bearings, "
                f"got {len(world_bearings)}")

        cos_bearings = np.cos(world_bearings)
        sin_bearings = np.sin(world_bearings)
        self._directions[:, :, 0] = (
            self._cos_elevations[:, None] * cos_bearings[None, :])
        self._directions[:, :, 1] = (
            self._cos_elevations[:, None] * sin_bearings[None, :])
        self._directions[:, :, 2] = np.sin(self._elevations)[:, None]

        # The steepest channel travels farther in 3D for the same horizontal
        # reach. Query that full slant range, then enforce the planar cutoff.
        slant_cutoff = (
            self._max_horizontal_range / np.min(self._cos_elevations))
        cast_world_collision_rays(
            model,
            data,
            origin,
            self._directions.reshape(-1),
            self._geom_groups,
            body_exclude,
            self._geom_ids,
            self._distances,
            self._ray_count,
            slant_cutoff,
        )

        ids = self._geom_ids.reshape(len(self._elevations), -1)
        horizontal = (
            self._distances.reshape(len(self._elevations), -1)
            * self._cos_elevations[:, None]
        )
        hit_height = (
            float(origin[2])
            + self._distances.reshape(len(self._elevations), -1)
            * np.sin(self._elevations)[:, None]
        )
        valid = (
            (ids >= 0)
            & (horizontal <= self._max_horizontal_range)
            & (hit_height >= NAV_LIDAR_MIN_OBSTACLE_Z_M)
            & (hit_height <= NAV_LIDAR_MAX_OBSTACLE_Z_M)
        )
        candidates = np.where(valid, horizontal, np.inf)
        winning_layer = np.argmin(candidates, axis=0)
        bearings = np.arange(self._bearing_count)
        ranges = candidates[winning_layer, bearings]
        hit_ids = np.where(
            np.isfinite(ranges), ids[winning_layer, bearings], -1)
        return ranges, hit_ids
