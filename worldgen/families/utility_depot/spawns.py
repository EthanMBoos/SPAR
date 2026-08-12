"""Deterministic default robot poses for utility-depot instances."""

from __future__ import annotations

import math
import random


def sample_spawn_defaults(seed: int) -> dict[str, dict[str, object]]:
    """Sample clear central defaults; an accepted brief may override them."""
    rng = random.Random(seed)
    husky_xy = rng.choice([
        (x, y)
        for x in (-2.0, 0.0, 2.0)
        for y in (-2.0, 0.0, 2.0)
    ])
    x2_offset = rng.choice(
        [
            (-3.0, 0.0), (-2.5, 2.5), (0.0, 3.0), (2.5, 2.5),
            (3.0, 0.0), (2.5, -2.5), (0.0, -3.0), (-2.5, -2.5),
        ]
    )
    cardinal_yaws = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    husky_yaw = rng.choice(cardinal_yaws)
    x2_yaw = rng.choice(cardinal_yaws)
    husky = {
        "position": [husky_xy[0], husky_xy[1], 0.0],
        "yaw": husky_yaw,
    }
    return {
        "husky_spawn": husky,
        "x2_spawn": {
            "position": [
                husky_xy[0] + x2_offset[0],
                husky_xy[1] + x2_offset[1],
                0.0,
            ],
            "yaw": x2_yaw,
        },
        # The default home is the Husky's initial pose. A supported brief may
        # place the dock elsewhere, and the exported autonomy config follows it.
        "dock": {
            "position": list(husky["position"]),
            "yaw": husky_yaw,
        },
    }
