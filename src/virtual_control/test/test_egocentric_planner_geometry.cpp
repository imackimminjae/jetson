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

TEST(EgocentricPlannerGeometry, RecordedPixelSliverDoesNotTriggerBranchTurnCost)
{
  // S1, ratio 0.7, first attempt, elapsed 18.326 s: k5 has a 0.1 m fragment.
  const std::vector<std::vector<double>> widths{
    {}, {10.5}, {10.5}, {10.6}, {11.7}, {12.6, 0.1}, {13.8}, {15.4}};
  const auto unfiltered = geometry::detectBranch(widths, 0.0);
  EXPECT_EQ(unfiltered.turn_step, 5);
  EXPECT_EQ(unfiltered.candidate_counts[5], 2);
  const auto filtered = geometry::detectBranch(widths, 2.0);
  EXPECT_EQ(filtered.candidate_counts[5], 1);
  EXPECT_EQ(filtered.turn_step, -1);
}

TEST(EgocentricPlannerGeometry, RecordedEntryWithTwoArmsThroughoutHasNoTurnWindow)
{
  // S1, ratio 0.7, first attempt, elapsed 21.426 s. All future stages have two arms.
  // k0->k1 is excluded; all remaining counts stay at two, so no new branch is detected.
  const std::vector<std::vector<double>> widths{
    {}, {9.6, 9.3}, {9.9, 9.0}, {10.3, 9.7}, {11.0, 8.4},
    {11.6, 6.8}, {11.9, 4.9}, {11.8, 15.0}};
  const auto branch = geometry::detectBranch(widths, 2.0);
  EXPECT_EQ(branch.candidate_counts[1], 2);
  EXPECT_EQ(branch.turn_step, -1);
}

TEST(EgocentricPlannerGeometry, BranchMinimumUsesRawWidthBeforeContraction)
{
  // 4 m observed runs have only 1.6 m preferred width at ratio 0.7.
  const auto branch = geometry::detectBranch({{}, {4.0}, {4.0, 4.0}}, 2.0);
  EXPECT_EQ(branch.candidate_counts[2], 2);
  EXPECT_EQ(branch.turn_step, 2);
}

TEST(EgocentricPlannerGeometry, BranchMinimumIncludesExactThreshold)
{
  EXPECT_EQ(geometry::detectBranch({{}, {2.0}, {2.0, 2.0}}, 2.0).turn_step, 2);
  const auto filtered = geometry::detectBranch({{}, {2.0}, {2.0, 1.99}}, 2.0);
  EXPECT_EQ(filtered.candidate_counts[2], 1);
  EXPECT_EQ(filtered.turn_step, -1);
}

TEST(EgocentricPlannerGeometry, CurrentStateCandidatesDoNotActivateFutureTurnWindow)
{
  const auto branch = geometry::detectBranch({{10.0, 10.0}, {10.0}, {10.0}}, 2.0);
  EXPECT_EQ(branch.candidate_counts[0], 2);
  EXPECT_EQ(branch.turn_step, -1);
}

TEST(EgocentricPlannerGeometry, CurrentStateToFirstPredictionIncreaseIsIgnored)
{
  const auto branch = geometry::detectBranch({{10.0}, {10.0, 10.0}, {10.0, 10.0}}, 2.0);
  EXPECT_EQ(branch.candidate_counts[0], 1);
  EXPECT_EQ(branch.candidate_counts[1], 2);
  EXPECT_EQ(branch.turn_step, -1);
}

TEST(EgocentricPlannerGeometry, FirstFutureIncreaseAnchorsTurnWindow)
{
  // A decrease or unchanged two-arm section is not a new split; 1->2 at k4 is.
  const auto branch = geometry::detectBranch(
    {{10.0}, {10.0, 10.0}, {10.0, 10.0}, {10.0}, {10.0, 10.0},
      {10.0, 10.0, 10.0}}, 2.0);
  EXPECT_EQ(branch.turn_step, 4);
}

TEST(EgocentricPlannerGeometry, TwoToThreeCandidatesAlsoCountsAsIncrease)
{
  EXPECT_EQ(geometry::detectBranch(
    {{10.0}, {10.0, 10.0}, {10.0, 10.0, 10.0}}, 2.0).turn_step, 2);
}

TEST(EgocentricPlannerGeometry, MissingEligiblePreviousSectionIsNotASplit)
{
  EXPECT_EQ(geometry::detectBranch({{10.0}, {}, {10.0, 10.0}}, 2.0).turn_step, -1);
}

TEST(EgocentricPlannerGeometry, IndependentBranchAblationsKeepOtherConditions)
{
  const std::vector<std::vector<double>> constant{{8.0}, {8.0, 8.0}, {8.0, 8.0}};
  EXPECT_EQ(geometry::detectBranch(constant, 2.0).turn_step, -1);
  // Restoring only first-multiple detection keeps k0->k1 excluded.
  EXPECT_EQ(geometry::detectBranch(constant, 2.0, false, false).turn_step, 2);
  EXPECT_EQ(geometry::detectBranch(constant, 2.0, true, true).turn_step, 1);
  // Including k0 does not turn a decrease into a split or invent missing data.
  EXPECT_EQ(geometry::detectBranch({{8.0, 8.0}, {8.0}}, 2.0, true, true).turn_step, -1);
  EXPECT_EQ(geometry::detectBranch({{}, {8.0, 8.0}}, 2.0, true, true).turn_step, -1);
}

TEST(EgocentricPlannerGeometry, OptionalContinuityRespectsWidthAndK0Exclusion)
{
  const std::vector<std::vector<double>> widths{{8.0}, {8.0, 8.0}, {8.0, 8.0}};
  const std::vector<std::vector<Eigen::Vector2d>> a{
    {{0, -4}}, {{3, -4}, {3, 1}}, {{6, -4}, {6, 1}}};
  const std::vector<std::vector<Eigen::Vector2d>> b{
    {{0, 4}}, {{3, -1}, {3, 4}}, {{6, -1}, {6, 4}}};
  EXPECT_EQ(geometry::detectBranchWithContinuity(a,b,widths,2.0,true,false,false,6.75).turn_step,-1);
  EXPECT_EQ(geometry::detectBranchWithContinuity(a,b,widths,2.0,true,false,true,6.75).turn_step,2);
  EXPECT_EQ(geometry::detectBranchWithContinuity(a,b,widths,2.0,true,true,true,6.75).turn_step,1);
  auto small=widths; small[1][1]=small[2][1]=.1;
  EXPECT_EQ(geometry::detectBranchWithContinuity(a,b,small,2.0,true,false,true,6.75).turn_step,-1);
  EXPECT_EQ(geometry::detectBranchWithContinuity(a,b,small,0.0,true,false,true,6.75).turn_step,2);
}
