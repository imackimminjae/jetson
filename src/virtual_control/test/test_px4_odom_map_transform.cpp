#include "virtual_control/px4_odom_map_transform.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>

namespace map_transform = virtual_control::px4_odom_map;

TEST(Px4OdomMapTransform, FirstSourcePoseMapsToEveryRouteAnchor)
{
  const map_transform::RawPose first_source{12.5, -7.25, 3.0, -0.42};

  for (int number = 1; number <= 21; ++number) {
    const auto route_name = "scenario" + std::to_string(number);
    const auto route = map_transform::routeAnchorForName(route_name);
    ASSERT_TRUE(route.has_value());
    map_transform::PoseAlignment alignment(map_transform::TransformConfig{}, *route);
    map_transform::MapPose output;

    ASSERT_TRUE(alignment.transform(first_source, output));
    EXPECT_NEAR(output.x, route->first[0], 1e-12);
    EXPECT_NEAR(output.y, route->first[1], 1e-12);
    EXPECT_NEAR(output.z, route->first[2], 1e-12);
    EXPECT_NEAR(output.yaw, route->targetYaw(), 1e-12);
  }
}

TEST(Px4OdomMapTransform, Scenario1RegressionSample)
{
  const auto route = map_transform::routeAnchorForName("scenario1");
  ASSERT_TRUE(route.has_value());
  map_transform::PoseAlignment alignment(map_transform::TransformConfig{}, *route);
  map_transform::MapPose output;

  // The first yaw follows from the supplied target/current/map yaw relation.
  const map_transform::RawPose first{
    54.9199448, 15.5843353, -1.40045166, -0.655140};
  ASSERT_TRUE(alignment.transform(first, output));

  const map_transform::RawPose subsequent{
    68.4597092, -7.4772625, 0.21081543, -0.626028};
  ASSERT_TRUE(alignment.transform(subsequent, output));

  EXPECT_NEAR(output.x, 147.043384, 1e-5);
  EXPECT_NEAR(output.y, -113.421420, 1e-5);
  EXPECT_NEAR(output.z, 1.611267, 1e-5);
  EXPECT_NEAR(output.yaw, 2.095460, 1e-6);
}

TEST(Px4OdomMapTransform, Scenario2StartsAtRoadCenterline)
{
  const auto route = map_transform::routeAnchorForName("scenario2");
  ASSERT_TRUE(route.has_value());

  EXPECT_NEAR(route->first[0], -252.789, 1e-12);
  EXPECT_NEAR(route->first[1], 93.548, 1e-12);
  EXPECT_NEAR(route->first[2], 0.0, 1e-12);
}

TEST(Px4OdomMapTransform, AxisSelectionMatchesPositionAndHeading)
{
  map_transform::TransformConfig config;
  config.position_scale = 2.0;
  config.x_offset = 3.0;
  config.y_offset = -4.0;
  config.swap_xy = true;
  config.invert_y = true;
  const auto route = map_transform::routeAnchorForName("scenario4");
  ASSERT_TRUE(route.has_value());
  map_transform::PoseAlignment alignment(config, *route);

  const auto source = alignment.transformSource({1.0, 2.0, 3.0, 0.0});
  EXPECT_NEAR(source.x, 7.0, 1e-12);
  EXPECT_NEAR(source.y, -6.0, 1e-12);
  EXPECT_NEAR(source.z, 6.0, 1e-12);
  EXPECT_NEAR(source.yaw, -0.5 * std::acos(-1.0), 1e-12);
}

TEST(Px4OdomMapTransform, OutputYawQuaternionHasUnitNorm)
{
  const auto quaternion = map_transform::yawQuaternion(2.095460);
  EXPECT_NEAR(quaternion.z * quaternion.z + quaternion.w * quaternion.w, 1.0, 1e-12);
}
