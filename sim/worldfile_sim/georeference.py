"""Small-world WGS84/ENU conversion and the MJCF datum contract."""

from __future__ import annotations

from dataclasses import dataclass
import math

import mujoco


MJCF_NUMERIC_NAME = "worldfile.world_georeference"
WGS84_SEMI_MAJOR_M = 6378137.0
WGS84_ECCENTRICITY_SQUARED = 6.69437999014e-3

# TODO: Accept an imported terrain/map datum so its geometry preserves real LLA.


@dataclass(frozen=True)
class Georeference:
    latitude_deg: float
    longitude_deg: float
    altitude_m: float

    def validate(self) -> None:
        values = (self.latitude_deg, self.longitude_deg, self.altitude_m)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("world georeference values must be finite")
        if not -90.0 < self.latitude_deg < 90.0:
            raise ValueError("world georeference latitude must be between the poles")
        if not -180.0 <= self.longitude_deg <= 180.0:
            raise ValueError("world georeference longitude is invalid")

    def _metres_per_radian(self) -> tuple[float, float]:
        latitude = math.radians(self.latitude_deg)
        sin_latitude = math.sin(latitude)
        denominator = math.sqrt(
            1.0 - WGS84_ECCENTRICITY_SQUARED * sin_latitude * sin_latitude
        )
        prime_vertical = WGS84_SEMI_MAJOR_M / denominator
        meridional = (
            WGS84_SEMI_MAJOR_M
            * (1.0 - WGS84_ECCENTRICITY_SQUARED)
            / (denominator ** 3)
        )
        east = (prime_vertical + self.altitude_m) * math.cos(latitude)
        north = meridional + self.altitude_m
        return east, north

    def enu_to_geodetic(
        self, east_m: float, north_m: float, up_m: float
    ) -> tuple[float, float, float]:
        east_scale, north_scale = self._metres_per_radian()
        return (
            self.latitude_deg + math.degrees(north_m / north_scale),
            self.longitude_deg + math.degrees(east_m / east_scale),
            self.altitude_m + up_m,
        )

    def geodetic_to_enu(
        self, latitude_deg: float, longitude_deg: float, altitude_m: float
    ) -> tuple[float, float, float]:
        east_scale, north_scale = self._metres_per_radian()
        return (
            math.radians(longitude_deg - self.longitude_deg) * east_scale,
            math.radians(latitude_deg - self.latitude_deg) * north_scale,
            altitude_m - self.altitude_m,
        )


def from_mjcf(model: mujoco.MjModel) -> Georeference:
    numeric_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_NUMERIC, MJCF_NUMERIC_NAME
    )
    if numeric_id < 0:
        raise ValueError(
            f"world is missing MJCF numeric {MJCF_NUMERIC_NAME!r}"
        )
    size = int(model.numeric_size[numeric_id])
    if size != 3:
        raise ValueError(
            f"MJCF numeric {MJCF_NUMERIC_NAME!r} must contain lat, lon, alt"
        )
    address = int(model.numeric_adr[numeric_id])
    values = model.numeric_data[address:address + size]
    result = Georeference(*(float(value) for value in values))
    result.validate()
    return result
