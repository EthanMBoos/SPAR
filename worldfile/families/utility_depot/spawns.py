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
    cardinal_yaws = (0.0, math.pi / 2.0, math.pi, -math.pi / 2.0)
    husky_yaw = rng.choice(cardinal_yaws)
    husky = {
        "position": [husky_xy[0], husky_xy[1], 0.0],
        "yaw": husky_yaw,
    }
    return {
        "husky_spawn": husky,
        # The mission needs a ground return/charging pose. It defaults to the
        # safe spawn but remains independently authorable by the world brief.
        "dock": {
            "position": list(husky["position"]),
            "yaw": husky_yaw,
        },
    }
