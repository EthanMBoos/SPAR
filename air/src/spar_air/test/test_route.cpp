#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <vector>

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

}  // namespace spar_air
