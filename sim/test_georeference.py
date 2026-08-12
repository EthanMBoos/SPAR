"""Host-side tests for the world datum and deterministic GPS errors."""

from __future__ import annotations

import statistics
import unittest

import mujoco

from spar_sim.georeference import Georeference, from_mjcf
from spar_sim.px4_link import (
    GPS_HORIZONTAL_STDDEV_M,
    GPS_VELOCITY_STDDEV_M_S,
    GPS_VERTICAL_STDDEV_M,
    GpsNoise,
)


class GeoreferenceTest(unittest.TestCase):
    def test_local_tangent_round_trip(self) -> None:
        datum = Georeference(33.7756, -84.3963, 300.0)
        for point in ((0.0, 0.0, 0.0), (-20.0, 20.0, 10.0), (20.0, -20.0, -1.0)):
            geodetic = datum.enu_to_geodetic(*point)
            recovered = datum.geodetic_to_enu(*geodetic)
            for actual, expected in zip(recovered, point):
                self.assertAlmostEqual(actual, expected, places=7)

    def test_committed_world_owns_datum(self) -> None:
        model = mujoco.MjModel.from_xml_path(
            "sim/worlds/utility_depot_40_v2.xml"
        )
        self.assertEqual(from_mjcf(model), Georeference(33.7756, -84.3963, 300.0))

    def test_gps_noise_is_repeatable_and_matches_declared_scale(self) -> None:
        first = GpsNoise(7)
        second = GpsNoise(7)
        self.assertEqual(
            first.sample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            second.sample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        )

        model = GpsNoise(11)
        samples = [
            model.sample((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            for _ in range(20_000)
        ]
        east = [sample[0][0] for sample in samples]
        up = [sample[0][2] for sample in samples]
        north_velocity = [sample[1][0] for sample in samples]
        self.assertAlmostEqual(
            statistics.stdev(east), GPS_HORIZONTAL_STDDEV_M, delta=0.01
        )
        self.assertAlmostEqual(
            statistics.stdev(up), GPS_VERTICAL_STDDEV_M, delta=0.02
        )
        self.assertAlmostEqual(
            statistics.stdev(north_velocity),
            GPS_VELOCITY_STDDEV_M_S,
            delta=0.01,
        )


if __name__ == "__main__":
    unittest.main()
