#include "virtual_control/egocentric_planner_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace geometry = imac_ctrl::egocentric_planner_geometry;

TEST(EgocentricPlannerGeometry, BinaryOccupancyThresholdZeroAndOneAreEquivalent)
{
  EXPECT_TRUE(geometry::gridValueIsDrivable(100, true, 0));
  EXPECT_TRUE(geometry::gridValueIsDrivable(100, true, 1));
  EXPECT_FALSE(geometry::gridValueIsDrivable(0, true, 0));
  EXPECT_FALSE(geometry::gridValueIsDrivable(0, true, 1));
}

TEST(EgocentricPlannerGeometry, NominalFallbackRejectsOnlyKnownBlockedReference)
{
  EXPECT_TRUE(geometry::referenceAllowsNominalFallback(false, -1, true, 0));
  EXPECT_TRUE(geometry::referenceAllowsNominalFallback(true, -1, true, 0));
  EXPECT_TRUE(geometry::referenceAllowsNominalFallback(true, 100, true, 0));
  EXPECT_FALSE(geometry::referenceAllowsNominalFallback(true, 0, true, 0));
}

TEST(EgocentricPlannerGeometry, ShortenedMatlabHorizonPersists)
{
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(5, 4U), 3);
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(5, 8U), 5);
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(5, 0U), 5);
}

TEST(EgocentricPlannerGeometry, FullHorizonRecoveryIsLimitedByCurrentGridNotPreviousPath)
{
  // A previously successful N=4 path has five points. A newly available fifth
  // step must be retried, but a smaller current grid still limits the request.
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(5, 5U, true), 5);
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(3, 6U, true), 3);
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(0, 5U, true), 0);
  EXPECT_EQ(geometry::retainMatlabPreviewHorizon(5, 0U, true), 5);
}

TEST(EgocentricPlannerGeometry, MissingCurrentStateIntervalKeepsPreviewHorizon)
{
  EXPECT_EQ(geometry::previewHorizonAfterMissingInterval(0, 5), 5);
}

TEST(EgocentricPlannerGeometry, MissingFutureIntervalKeepsOnlyContiguousPrefix)
{
  EXPECT_EQ(geometry::previewHorizonAfterMissingInterval(1, 5), 0);
  EXPECT_EQ(geometry::previewHorizonAfterMissingInterval(3, 5), 2);
  EXPECT_EQ(geometry::previewHorizonAfterMissingInterval(8, 5), 5);
}

TEST(EgocentricPlannerGeometry, FirstMovingCycleExposesFollowingSegment)
{
  const std::vector<Eigen::Vector2d> route{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(20.0, 0.0),
    Eigen::Vector2d(20.0, 20.0)};

  const auto pair = geometry::findWaypointPair(
    route, 0, route.front(), Eigen::Vector2d(1.0, 0.0), 10.0, 10.0);

  EXPECT_EQ(pair[0], 1);
  EXPECT_EQ(pair[1], 2);
}

TEST(EgocentricPlannerGeometry, StationaryPoseDoesNotInventMovingDirection)
{
  const std::vector<Eigen::Vector2d> route{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(20.0, 0.0),
    Eigen::Vector2d(20.0, 20.0)};

  const auto pair = geometry::findWaypointPair(
    route, 0, route.front(), Eigen::Vector2d::Zero(), 10.0, 10.0);

  EXPECT_EQ(pair[0], 0);
  EXPECT_EQ(pair[1], 1);
}

TEST(EgocentricPlannerGeometry, RouteEndRemainsFinalForwardPair)
{
  const std::vector<Eigen::Vector2d> route{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(20.0, 0.0),
    Eigen::Vector2d(20.0, 20.0)};

  const auto pair = geometry::findWaypointPair(
    route, 1, route[1], Eigen::Vector2d(0.0, 1.0), 10.0, 10.0);

  EXPECT_EQ(pair[0], 1);
  EXPECT_EQ(pair[1], 2);
}

TEST(EgocentricPlannerGeometry, LateralMissDoesNotAdvanceWaypoint)
{
  const std::vector<Eigen::Vector2d> route{
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(20.0, 0.0),
    Eigen::Vector2d(40.0, 0.0)};

  const auto pair = geometry::findWaypointPair(
    route, 1, Eigen::Vector2d(20.0, 15.0), Eigen::Vector2d(1.0, 0.0),
    10.0, 10.0);

  EXPECT_EQ(pair[0], 1);
  EXPECT_EQ(pair[1], 2);
}

TEST(EgocentricPlannerGeometry, TerminalTargetUsesWp0WhileOneStepAhead)
{
  const Eigen::Vector2d terminal = geometry::buildOneStepTerminalTarget(
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(3.0, 0.0),
    Eigen::Vector2d(10.0, 0.0), 1.0, Eigen::Vector2d::UnitX());

  EXPECT_NEAR(terminal.x(), 1.0, 1e-12);
  EXPECT_NEAR(terminal.y(), 0.0, 1e-12);
}

TEST(EgocentricPlannerGeometry, TerminalTargetSwitchesToWp1NearWp0)
{
  const Eigen::Vector2d previous(2.5, 0.0);
  const Eigen::Vector2d terminal = geometry::buildOneStepTerminalTarget(
    previous, Eigen::Vector2d(3.0, 0.0), Eigen::Vector2d(10.0, 0.0),
    1.0, Eigen::Vector2d::UnitX());

  EXPECT_NEAR((terminal - previous).norm(), 1.0, 1e-12);
  EXPECT_GT(terminal.x(), 3.0);
  EXPECT_NEAR(terminal.y(), 0.0, 1e-12);
}

TEST(EgocentricPlannerGeometry, TerminalSelectionUsesForwardProgressNotEuclideanDistance)
{
  const Eigen::Vector2d previous(0.0, 0.0);
  const Eigen::Vector2d terminal = geometry::buildOneStepTerminalTarget(
    previous, Eigen::Vector2d(0.5, 10.0), Eigen::Vector2d(5.0, 0.0),
    1.0, Eigen::Vector2d::UnitX());

  EXPECT_NEAR(terminal.x(), 1.0, 1e-12);
  EXPECT_NEAR(terminal.y(), 0.0, 1e-12);
}

TEST(EgocentricPlannerGeometry, PreviousPathUsesCurrentMeasuredBodyFrame)
{
  const std::vector<Eigen::Vector2d> previous_world{
    Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(3.0, 0.0),
    Eigen::Vector2d(6.0, 0.0), Eigen::Vector2d(9.0, 0.0),
    Eigen::Vector2d(12.0, 0.0), Eigen::Vector2d(15.0, 0.0)};
  std::vector<Eigen::Vector2d> shifted;

  ASSERT_TRUE(
    geometry::buildMeasuredFrameShiftedPrefix(
      previous_world, 0.5 * std::acos(-1.0), 5, shifted));
  ASSERT_EQ(shifted.size(), 6U);
  EXPECT_NEAR(shifted[0].norm(), 0.0, 1e-12);
  EXPECT_NEAR(shifted[1].x(), 0.0, 1e-12);
  EXPECT_NEAR(shifted[1].y(), -3.0, 1e-12);
  EXPECT_NEAR(shifted[3].y(), -9.0, 1e-12);
  // Old pN must not leak into the next reference prefix.
  EXPECT_NEAR(shifted[4].norm(), 0.0, 1e-12);
  EXPECT_NEAR(shifted[5].norm(), 0.0, 1e-12);
}

TEST(EgocentricPlannerGeometry, PreviewExtentKeepsOneStepInsideBev)
{
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(19.73, 3.0, 7), 5);
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(2.0, 3.0, 7), 0);
}

TEST(EgocentricPlannerGeometry, FifteenMetrePreviewFitsShiftedBevWithoutFullStepReserve)
{
  // Vehicle x=[-2.5,17.5]: the last of five 3 m steps is at 15 m.
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(17.5, 3.0, 5, 0), 5);
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(17.5, 3.0, 5, 1), 4);
}

TEST(EgocentricPlannerGeometry, ZeroReserveStillHonoursExtentAndStepCap)
{
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(14.9, 3.0, 5, 0), 4);
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(2.9, 3.0, 5, 0), 0);
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(22.5, 3.0, 5, 0), 5);
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(-2.5, 3.0, 5, 0), 0);
}

TEST(EgocentricPlannerGeometry, NegativeReserveCannotExtendPreviewBeyondBev)
{
  EXPECT_EQ(geometry::previewStepsFromForwardExtent(17.5, 3.0, 5, -1), 0);
}
