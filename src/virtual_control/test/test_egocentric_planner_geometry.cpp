#include "virtual_control/egocentric_planner_geometry.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace geometry = imac_ctrl::egocentric_planner_geometry;

TEST(EgocentricPlannerGeometry, WaypointBiasWindowUsesInclusiveOneBasedWp0)
{
  EXPECT_FALSE(geometry::waypointBiasWindowActive(1, 3, 4));
  EXPECT_TRUE(geometry::waypointBiasWindowActive(2, 3, 4));
  EXPECT_TRUE(geometry::waypointBiasWindowActive(3, 3, 4));
  EXPECT_FALSE(geometry::waypointBiasWindowActive(4, 3, 4));
  EXPECT_FALSE(geometry::waypointBiasWindowActive(0, 0, 4));
}

TEST(EgocentricPlannerGeometry, LocalBiasRotatesAboutEgoOnlyInsideWindow)
{
  const Eigen::Vector2d waypoint(20.0, 0.0);
  const auto perturbed = geometry::biasPlanningWaypointBody(
    waypoint, std::acos(-1.0) / 2.0, Eigen::Vector2d::Zero(), true);
  EXPECT_NEAR(perturbed.x(), 0.0, 1e-12);
  EXPECT_NEAR(perturbed.y(), 20.0, 1e-12);
  EXPECT_TRUE(geometry::biasPlanningWaypointBody(
    waypoint, 1.0, Eigen::Vector2d(5.0, 5.0), false).isApprox(waypoint));
  EXPECT_TRUE(waypoint.isApprox(Eigen::Vector2d(20.0, 0.0)));
}

TEST(EgocentricPlannerGeometry, LocalBiasDoesNotMoveWaypointProgressGate)
{
  const std::vector<Eigen::Vector2d> route{{0.0, 0.0}, {200.0, 0.0},
    {220.0, 0.0}, {220.0, 50.0}};
  const Eigen::Vector2d ego(200.0, 0.0);
  const auto target = geometry::biasPlanningWaypointBody(
    route[2] - ego, -5.0 * std::acos(-1.0) / 180.0, Eigen::Vector2d::Zero(), true);
  EXPECT_NEAR(target.norm(), 20.0, 1e-12);
  const auto pair = geometry::findWaypointPair(route, 1, ego, Eigen::Vector2d(1.0, 0.0), 10.0, 15.0);
  EXPECT_EQ(pair[0], 2);
  EXPECT_TRUE(route[1].isApprox(ego));
}

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

TEST(EgocentricPlannerGeometry, PreviousPathAdvancesOnlyByMeasuredProgress)
{
  const std::vector<Eigen::Vector2d> old{{0, 0}, {5.4, 0}, {10.8, 0},
    {10.8, 5.4}, {10.8, 10.8}, {10.8, 16.2}};
  std::vector<Eigen::Vector2d> prefix;
  const Eigen::Vector2d ego(1.5, 0.2);
  ASSERT_TRUE(geometry::buildMeasuredFrameProgressPrefix(old, ego, 0.0, 5.4, 5, prefix));
  EXPECT_NEAR(prefix[0].norm(), 0, 1e-12);
  EXPECT_NEAR((prefix[1] + ego - Eigen::Vector2d(6.9, 0)).norm(), 0, 1e-12);
  EXPECT_NEAR((prefix[2] + ego - Eigen::Vector2d(10.8, 1.5)).norm(), 0, 1e-12);
  // Repeat at an unchanged pose: do not consume another whole segment.
  std::vector<Eigen::Vector2d> repeated;
  ASSERT_TRUE(geometry::buildMeasuredFrameProgressPrefix(old, ego, 0.0, 5.4, 5, repeated));
  EXPECT_NEAR((prefix[2] - repeated[2]).norm(), 0, 1e-12);
  ASSERT_TRUE(geometry::buildMeasuredFrameProgressPrefix(old, ego, std::acos(-1.0)/2, 5.4, 5, repeated));
  EXPECT_NEAR(repeated[1].x(), -0.2, 1e-12);
  EXPECT_NEAR(repeated[1].y(), -5.4, 1e-12);
}

TEST(EgocentricPlannerGeometry, ProgressPrefixHandlesDuplicatesAndTailWithoutJumping)
{
  std::vector<Eigen::Vector2d> prefix;
  ASSERT_TRUE(geometry::buildMeasuredFrameProgressPrefix(
    {{0, 0}, {0, 0}, {5, 0}}, {4, 0}, 0, 2, 5, prefix));
  EXPECT_NEAR(prefix[1].x(), 2, 1e-12);
  EXPECT_NEAR(prefix[3].x(), 6, 1e-12);
  EXPECT_FALSE(geometry::buildMeasuredFrameProgressPrefix({{0, 0}, {0, 0}}, {0, 0}, 0, 2, 5, prefix));
  EXPECT_FALSE(geometry::buildMeasuredFrameProgressPrefix({{0, 0}, {5, 0}}, {NAN, 0}, 0, 2, 5, prefix));
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

TEST(EgocentricPlannerGeometry, RecordedPixelSliverStillCountsAsAnInterval)
{
  // S1 at 18.326 s: the 0.1 m fragment at k5 remains a second interval.
  EXPECT_EQ(geometry::firstBranchStep({0, 1, 1, 1, 1, 2, 1, 1}), 5);
}

TEST(EgocentricPlannerGeometry, RecordedTwoArmEntryStartsAtSecondPrediction)
{
  // S1 at 21.426 s: all future sections have two arms, including k1.
  EXPECT_EQ(geometry::firstBranchStep({0, 2, 2, 2, 2, 2, 2, 2}), 2);
}

TEST(EgocentricPlannerGeometry, CurrentStateAndK1DoNotAnchorTurnGuidance)
{
  EXPECT_EQ(geometry::firstBranchStep({2, 1, 1}), -1);
  EXPECT_EQ(geometry::firstBranchStep({3}), -1);
  EXPECT_EQ(geometry::firstBranchStep({}), -1);
  EXPECT_EQ(geometry::firstBranchStep({1, 2, 1}), -1);
  EXPECT_EQ(geometry::firstBranchStep({1, 2}), -1);
}

TEST(EgocentricPlannerGeometry, CountsNeedNotIncreaseToActivateGuidance)
{
  EXPECT_EQ(geometry::firstBranchStep({3, 2, 2}), 2);
  EXPECT_EQ(geometry::firstBranchStep({1, 2, 3}), 2);
  EXPECT_EQ(geometry::firstBranchStep({0, 1, 3, 2}), 2);
}

TEST(EgocentricPlannerGeometry, FirstMultipleSectionWins)
{
  EXPECT_EQ(geometry::firstBranchStep({1, 1, 1, 2, 1, 3}), 3);
  EXPECT_EQ(geometry::firstBranchStep({1, 1, 1, 1}), -1);
}
