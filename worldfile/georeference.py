"""Repository-owned geographic references for generated world families."""

from __future__ import annotations

import math
from typing import Any


UTILITY_DEPOT_GEOREFERENCE: dict[str, Any] = {
    "frame": "WGS84",
    "latitude_deg": 33.7756,
    "longitude_deg": -84.3963,
    "altitude_m": 300.0,
    "altitude_reference": "simulated_msl",
    "map_frame": "ENU",
    "projection": "wgs84_local_tangent",
}

# TODO: Accept real terrain/map CRS and datum metadata, then import it into
# this world-aligned local ENU frame.


def family_georeference(family: str) -> dict[str, Any]:
    """Return a copy of the fixed geographic reference for one family."""
    # Every currently supported fictional family shares the campus datum.
    # The family argument leaves an explicit extension point for a future
    # real-site family without making geography an LLM or seed decision.
    del family
    return dict(UTILITY_DEPOT_GEOREFERENCE)


def valid_georeference(value: Any) -> bool:
    if not isinstance(value, dict):
        return False
    numeric = (
        value.get("latitude_deg"),
        value.get("longitude_deg"),
        value.get("altitude_m"),
    )
    return (
        all(
            isinstance(item, (int, float))
            and not isinstance(item, bool)
            and math.isfinite(item)
            for item in numeric
        )
        and -90.0 < numeric[0] < 90.0
        and -180.0 <= numeric[1] <= 180.0
        and value.get("frame") == "WGS84"
        and value.get("altitude_reference") == "simulated_msl"
        and value.get("map_frame") == "ENU"
        and value.get("projection") == "wgs84_local_tangent"
    )
