#include "virtual_control/interval_centering_state.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace centering = imac_ctrl::interval_centering;
using Preview = std::vector<std::vector<centering::Observation>>;

namespace
{
centering::Observation observed(double length)
{
  return centering::Observation{length, true, true, true, false};
}

Preview nearPair(double first, double second)
{
  return {{observed(first)}, {observed(second)}};
}
}  // namespace

TEST(IntervalCenteringState, InitializesFromNearestAgreeingPairIncludingK0)
{
  const Preview preview{{observed(4.0)}, {observed(4.4)}, {observed(4.8)}};
  const auto info = centering::update({}, preview);
  EXPECT_EQ(info.reason, centering::UpdateReason::Initialized);
  EXPECT_EQ(info.candidate_first_step, 0);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.2);
  EXPECT_TRUE(info.state.confirmation_lengths.empty());
}

TEST(IntervalCenteringState, ChoosesK1K2WhenFirstPairDisagrees)
{
  const Preview preview{{observed(10.0)}, {observed(4.0)}, {observed(4.2)}};
  const auto info = centering::update({}, preview);
  EXPECT_EQ(info.candidate_first_step, 1);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.1);
}

TEST(IntervalCenteringState, NeverBridgesMissingNodeOrUsesFarPair)
{
  const Preview preview{{observed(4.0)}, {}, {observed(4.0)}, {observed(4.0)}};
  const auto info = centering::update({}, preview);
  EXPECT_FALSE(centering::hasReference(info.state));
  EXPECT_EQ(info.reason, centering::UpdateReason::NoCandidate);
}

TEST(IntervalCenteringState, RejectsEachUnreliableNearPairCondition)
{
  for (int condition = 0; condition < 6; ++condition) {
    auto preview = nearPair(4.0, 4.0);
    auto & first = preview[0][0];
    if (condition == 0) {first.start_boundary_observed = false;}
    if (condition == 1) {first.end_boundary_observed = false;}
    if (condition == 2) {first.nominal_fallback = true;}
    if (condition == 3) {first.reference_inside = false;}
    if (condition == 4) {preview[0].push_back(observed(0.5));}
    if (condition == 5) {first.length = std::numeric_limits<double>::quiet_NaN();}
    EXPECT_FALSE(centering::hasReference(centering::update({}, preview).state));
  }
}

TEST(IntervalCenteringState, PairAgreementBoundaryIsInclusive)
{
  EXPECT_TRUE(centering::hasReference(centering::update({}, nearPair(10.0, 12.0)).state));
  EXPECT_FALSE(centering::hasReference(centering::update({}, nearPair(10.0, 12.01)).state));
}

TEST(IntervalCenteringState, InitializationPrecedesBranchFreeze)
{
  auto preview = nearPair(4.0, 4.0);
  preview.push_back({observed(2.0), observed(2.0)});
  const auto info = centering::update({}, preview);
  EXPECT_TRUE(info.multiple_observed_intervals);
  EXPECT_EQ(info.reason, centering::UpdateReason::Initialized);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
}

TEST(IntervalCenteringState, BranchAnywhereFreezesExistingScaleAndClearsHistory)
{
  auto preview = nearPair(8.0, 8.0);
  preview.push_back({});
  preview.push_back({observed(2.0), observed(3.0)});
  const auto info = centering::update({4.0, {8.0, 8.0}}, preview);
  EXPECT_EQ(info.reason, centering::UpdateReason::MultipleObservedIntervals);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
  EXPECT_TRUE(info.state.confirmation_lengths.empty());
}

TEST(IntervalCenteringState, SyntheticIntervalsNeverInitializeScaleOrCountAsBranch)
{
  auto fallback = observed(4.5);
  fallback.nominal_fallback = true;
  EXPECT_FALSE(centering::hasReference(centering::update({}, {{fallback}, {fallback}}).state));
  auto preview = nearPair(4.0, 4.0);
  preview.push_back({fallback, fallback});
  const auto info = centering::update({4.0, {}}, preview);
  EXPECT_FALSE(info.multiple_observed_intervals);
  EXPECT_EQ(info.reason, centering::UpdateReason::Confirming);
}

TEST(IntervalCenteringState, ObservationLossRetainsScaleAndRestartsConfirmation)
{
  const auto lost = centering::update({4.0, {8.0, 8.0}}, {});
  EXPECT_DOUBLE_EQ(lost.state.reference_length, 4.0);
  EXPECT_TRUE(lost.state.confirmation_lengths.empty());
  auto next = centering::update(lost.state, nearPair(8.0, 8.0));
  EXPECT_EQ(next.reason, centering::UpdateReason::Confirming);
  EXPECT_DOUBLE_EQ(next.state.reference_length, 4.0);
  EXPECT_EQ(next.state.confirmation_lengths.size(), 1U);
}

TEST(IntervalCenteringState, ThreeConsistentCandidatesUseMedianAndBoundIncrease)
{
  centering::State state{4.0, {}};
  auto first = centering::update(state, nearPair(8.0, 8.0));
  auto second = centering::update(first.state, nearPair(9.0, 9.0));
  auto third = centering::update(second.state, nearPair(8.5, 8.5));
  EXPECT_DOUBLE_EQ(first.state.reference_length, 4.0);
  EXPECT_DOUBLE_EQ(second.state.reference_length, 4.0);
  EXPECT_EQ(third.reason, centering::UpdateReason::Updated);
  EXPECT_DOUBLE_EQ(third.update_target, 8.5);
  EXPECT_DOUBLE_EQ(third.state.reference_length, 4.0 * 1.05);
  const auto fourth = centering::update(third.state, nearPair(8.5, 8.5));
  EXPECT_EQ(fourth.state.confirmation_lengths.size(), 3U);
  EXPECT_NEAR(fourth.state.reference_length, 4.0 * 1.05 * 1.05, 1e-12);
}

TEST(IntervalCenteringState, DecreaseUsesReciprocalBoundNotFivePercentSubtraction)
{
  const auto info = centering::update({4.0, {1.0, 1.0}}, nearPair(1.0, 1.0));
  EXPECT_EQ(info.reason, centering::UpdateReason::Updated);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0 / 1.05);
  EXPECT_GT(info.state.reference_length, 4.0 * 0.95);
}

TEST(IntervalCenteringState, CloseTargetUsesMedianWithoutArtificialFullStep)
{
  const auto info = centering::update({4.0, {4.05, 4.10}}, nearPair(4.15, 4.15));
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.10);
}

TEST(IntervalCenteringState, InconsistentHistoryHoldsUntilThreeConsecutiveAgree)
{
  auto info = centering::update({4.0, {2.0, 8.0}}, nearPair(8.0, 8.0));
  EXPECT_EQ(info.reason, centering::UpdateReason::CandidatesDisagree);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
  info = centering::update(info.state, nearPair(8.0, 8.0));
  EXPECT_EQ(info.reason, centering::UpdateReason::Updated);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.2);
}

TEST(IntervalCenteringState, PreviewMinimumNeverReplacesEstablishedScale)
{
  auto preview = nearPair(4.0, 4.0);
  preview.push_back({observed(0.1)});
  const auto info = centering::update({4.0, {}}, preview);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
  EXPECT_DOUBLE_EQ(info.candidate_length, 4.0);
}

TEST(IntervalCenteringState, OnOffUsesInclusiveRelativeThresholdAndFixedContraction)
{
  const auto on = centering::treatment(observed(6.0), {4.0, {}}, {}, 0.7, 2.0, 9.0);
  EXPECT_TRUE(on.centering_on);
  EXPECT_DOUBLE_EQ(on.threshold, 6.0);
  EXPECT_NEAR(6.0 - 2.0 * on.inset, 2.4, 1e-12);
  const auto off = centering::treatment(observed(6.01), {4.0, {}}, {}, 0.7, 2.0, 9.0);
  EXPECT_FALSE(off.centering_on);
  EXPECT_DOUBLE_EQ(off.inset, 2.0);
}

TEST(IntervalCenteringState, UninitializedObservedIntervalsAreOffEvenWhenShort)
{
  const auto off = centering::treatment(observed(1.0), {}, {}, 0.7, 2.0, 9.0);
  EXPECT_FALSE(off.centering_on);
  EXPECT_TRUE(std::isnan(off.threshold));
  EXPECT_DOUBLE_EQ(off.inset, 0.45);
  EXPECT_NEAR(1.0 - 2.0 * off.inset, 0.1, 1e-12);
  const auto on = centering::treatment(observed(1.0), {4.0, {}}, {}, 0.7, 2.0, 9.0);
  EXPECT_GT(off.inset, on.inset);  // OFF may be narrower than ON, as specified.
}

TEST(IntervalCenteringState, FallbackHasIndependentNineMetreThreshold)
{
  auto fallback = observed(4.5);
  fallback.nominal_fallback = true;
  for (const centering::State state : {centering::State{}, centering::State{1.0, {}}}) {
    const auto decision = centering::treatment(fallback, state, {}, 0.7, 2.0, 9.0);
    EXPECT_TRUE(decision.centering_on);
    EXPECT_DOUBLE_EQ(decision.threshold, 9.0);
    EXPECT_NEAR(4.5 - 2.0 * decision.inset, 1.8, 1e-12);
  }
  fallback.length = 9.01;
  EXPECT_FALSE(centering::treatment(fallback, {}, {}, 0.7, 2.0, 9.0).centering_on);
  EXPECT_FALSE(centering::treatment(observed(4.5), {1.0, {}}, {}, 0.7, 2.0, 9.0).centering_on);
}
