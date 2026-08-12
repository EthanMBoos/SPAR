#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

#include "frames.hpp"
#include "route.hpp"

namespace spar_air {

TEST(AirRoute, ParsesExplicitXyzyawTuples) {
  const auto route = parsePatrolWaypoints({
      1.0, 2.0, 4.0, 0.1,
      3.0, 4.0, 5.0, 0.2,
      5.0, 6.0, 6.0, 0.3,
  });
  ASSERT_EQ(route.size(), 3u);
  EXPECT_DOUBLE_EQ(route[1].z, 5.0);
  EXPECT_DOUBLE_EQ(route[2].yaw, 0.3);
}

TEST(AirRoute, RejectsMissingOrMalformedRoutes) {
  EXPECT_THROW(parsePatrolWaypoints({}), std::runtime_error);
  EXPECT_THROW(
      parsePatrolWaypoints({1.0, 2.0, 4.0, 0.0, 3.0}),
      std::runtime_error);
}

TEST(AirRoute, RejectsInvalidValues) {
  EXPECT_THROW(parsePatrolWaypoints({
      1.0, 2.0, 0.0, 0.0,
      3.0, 4.0, 4.0, 0.0,
      5.0, 6.0, 4.0, 0.0,
  }), std::runtime_error);
  EXPECT_THROW(parsePatrolWaypoints({
      1.0, 2.0, 4.0, 0.0,
      3.0, 4.0, 4.0, std::numeric_limits<double>::infinity(),
      5.0, 6.0, 4.0, 0.0,
  }), std::runtime_error);
}

TEST(AirFrames, UsesPx4ReportedGlobalReference) {
  const spar_geodesy::Geodetic datum{33.7756, -84.3963, 300.0};
  const spar_geodesy::LocalTangent world(datum);
  const auto reference = world.enuToGeodetic({-2.0, -1.0, 0.1});
  Px4MapTransform transform(datum);

  EXPECT_FALSE(transform.ready());
  const auto update = transform.updateReference(
      true, true, true, true, 100, reference.latitude_deg,
      reference.longitude_deg, reference.altitude_m, 0, 0);
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.reference_changed);

  const auto map = transform.localNedToMap({4.0, 3.0, -5.0});
  ASSERT_TRUE(map.has_value());
  EXPECT_NEAR(map->x, 1.0, 1e-7);
  EXPECT_NEAR(map->y, 3.0, 1e-7);
  EXPECT_NEAR(map->z, 5.1, 1e-7);

  const auto local = transform.mapToLocalNed(*map);
  ASSERT_TRUE(local.has_value());
  EXPECT_NEAR(local->x, 4.0, 1e-7);
  EXPECT_NEAR(local->y, 3.0, 1e-7);
  EXPECT_NEAR(local->z, -5.0, 1e-7);
}

TEST(AirFrames, InvalidPositionSuppressesPoseAndSetpointConversion) {
  Px4MapTransform transform({33.7756, -84.3963, 300.0});
  transform.updateReference(
      false, false, true, true, 1, 33.7756, -84.3963, 300.0, 0, 0);
  EXPECT_TRUE(transform.hasReference());
  EXPECT_FALSE(transform.ready());
  EXPECT_FALSE(transform.localNedToMap({0.0, 0.0, 0.0}).has_value());
  EXPECT_FALSE(transform.mapToLocalNed({0.0, 0.0, 0.0}).has_value());
}

TEST(AirFrames, LostGlobalReferenceSuppressesPoseAndSetpointConversion) {
  Px4MapTransform transform({33.7756, -84.3963, 300.0});
  transform.updateReference(
      true, true, true, true, 1, 33.7756, -84.3963, 300.0, 0, 0);
  ASSERT_TRUE(transform.ready());

  transform.updateReference(
      true, true, false, false, 1, 33.7756, -84.3963, 300.0, 0, 0);
  EXPECT_FALSE(transform.ready());
  EXPECT_FALSE(transform.localNedToMap({0.0, 0.0, 0.0}).has_value());
  EXPECT_FALSE(transform.mapToLocalNed({0.0, 0.0, 0.0}).has_value());
}

TEST(AirFrames, DetectsEstimatorResetAndKeepsMapGoalInvariant) {
  const spar_geodesy::Geodetic datum{33.7756, -84.3963, 300.0};
  const spar_geodesy::LocalTangent world(datum);
  Px4MapTransform transform(datum);
  transform.updateReference(true, true, true, true, 1,
                            datum.latitude_deg, datum.longitude_deg,
                            datum.altitude_m, 0, 0);
  const Vec3 goal{8.0, 6.0, 4.0};
  const auto before = transform.mapToLocalNed(goal);
  ASSERT_TRUE(before.has_value());

  const auto shifted = world.enuToGeodetic({1.0, -2.0, 0.5});
  const auto update = transform.updateReference(
      true, true, true, true, 2, shifted.latitude_deg,
      shifted.longitude_deg, shifted.altitude_m, 1, 1);
  EXPECT_TRUE(update.reference_changed);
  EXPECT_TRUE(update.xy_reset);
  EXPECT_TRUE(update.z_reset);
  const auto after = transform.mapToLocalNed(goal);
  ASSERT_TRUE(after.has_value());
  EXPECT_NEAR(after->x, before->x + 2.0, 1e-7);
  EXPECT_NEAR(after->y, before->y - 1.0, 1e-7);
  EXPECT_NEAR(after->z, before->z + 0.5, 1e-7);
}

TEST(AirFrames, NormalizesYawAndConvertsQuaternionBasis) {
  EXPECT_NEAR(yawEnuToNed(-spar_geodesy::kPi),
              -0.5 * spar_geodesy::kPi, 1e-12);
  EXPECT_NEAR(yawEnuToNed(0.0), 0.5 * spar_geodesy::kPi, 1e-12);
  const auto converted = frdNedToFluEnu({1.0, 0.0, 0.0, 0.0});
  EXPECT_NEAR(std::abs(converted[0]), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(std::abs(converted[3]), std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(converted[1], 0.0, 1e-12);
  EXPECT_NEAR(converted[2], 0.0, 1e-12);
}

}  // namespace spar_air
