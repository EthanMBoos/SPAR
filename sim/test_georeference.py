"""Host-side tests for the world-owned geographic datum."""

from __future__ import annotations

import unittest

import mujoco

from spar_sim.georeference import Georeference, from_mjcf


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


if __name__ == "__main__":
    unittest.main()
