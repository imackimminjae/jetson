#include "virtual_control/tracking_control.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace imac_ctrl
{
// Access the planner's private helpers without constructing or spinning a node.
struct IntervalCenteringTestAccess
{
  using Observation = SdMapUpperPlannerNode::IntervalObservation;
  using State = SdMapUpperPlannerNode::IntervalCenteringState;
  using Settings = SdMapUpperPlannerNode::IntervalCenteringSettings;

  static bool hasReference(const State & state)
  {
    return SdMapUpperPlannerNode::hasIntervalReference(state);
  }

  static auto update(
    const State & previous, const std::vector<std::vector<Observation>> & preview,
    const Settings & settings = Settings{})
  {
    return SdMapUpperPlannerNode::updateIntervalCentering(previous, preview, settings);
  }

  static auto treatment(
    const Observation & interval, const State & state, const Settings & settings,
    double soft_ratio, double boundary_margin, double fallback_max_centering_length)
  {
    return SdMapUpperPlannerNode::intervalTreatment(
      interval, state, settings, soft_ratio, boundary_margin, fallback_max_centering_length);
  }
};
}  // namespace imac_ctrl

using centering = imac_ctrl::IntervalCenteringTestAccess;
using Preview = std::vector<std::vector<centering::Observation>>;

namespace
{
centering::Observation observed(double length, bool inside = true)
{
  return centering::Observation{length, true, true, inside, false};
}
}  // namespace

TEST(IntervalCenteringState, FarNarrowRoadImmediatelyReleasesLongEntryIntervals)
{
  const Preview preview{{observed(10.0)}, {observed(10.2)}, {}, {observed(9.0)},
    {observed(8.0)}, {observed(4.8)}};
  for (const centering::State previous : {centering::State{}, centering::State{10.0}}) {
    const auto info = centering::update(previous, preview);
    EXPECT_DOUBLE_EQ(info.previous_reference_length, previous.reference_length);
    EXPECT_DOUBLE_EQ(info.candidate_length, 4.8);
    EXPECT_DOUBLE_EQ(info.state.reference_length, 4.8);
    EXPECT_EQ(info.candidate_step, 5);
    EXPECT_EQ(info.candidate_interval, 0);
    for (const auto & intervals : preview) {
      for (const auto & interval : intervals) {
        const auto decision = centering::treatment(interval, info.state, {}, 0.7, 2.0, 9.0);
        EXPECT_NEAR(decision.threshold, 7.2, 1e-12);
        EXPECT_EQ(decision.centering_on, interval.length <= 7.2);
        EXPECT_NEAR(decision.inset, decision.centering_on ? 0.3 * interval.length : 2.0, 1e-12);
      }
    }
  }
}

TEST(IntervalCenteringState, IncludesK0AndSelectsOriginalLength)
{
  const auto info = centering::update({}, {{observed(4.0)}, {observed(4.4)}, {observed(4.8)}});
  EXPECT_EQ(info.candidate_step, 0);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
}

TEST(IntervalCenteringState, OtherBranchesNeitherSupplyCandidateNorFreezeUpdate)
{
  const Preview preview{{observed(10.0)}, {},
    {observed(0.5, false), observed(4.8), observed(1.0, false)}};
  const auto info = centering::update({10.0}, preview);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.8);
  EXPECT_EQ(info.candidate_step, 2);
  EXPECT_EQ(info.candidate_interval, 1);
  EXPECT_FALSE(centering::hasReference(centering::update({}, {{observed(0.5, false)}}).state));
}

TEST(IntervalCenteringState, RejectsClippedFallbackNonfiniteAndNonpositiveCandidates)
{
  for (int condition = 0; condition < 8; ++condition) {
    auto interval = observed(4.0);
    if (condition == 0) {interval.start_boundary_observed = false;}
    if (condition == 1) {interval.end_boundary_observed = false;}
    if (condition == 2) {interval.nominal_fallback = true;}
    if (condition == 3) {interval.reference_inside = false;}
    if (condition == 4) {interval.length = std::numeric_limits<double>::quiet_NaN();}
    if (condition == 5) {interval.length = std::numeric_limits<double>::infinity();}
    if (condition == 6) {interval.length = 0.0;}
    if (condition == 7) {interval.length = -1.0;}
    const auto absent = centering::update({}, {{interval}});
    EXPECT_FALSE(centering::hasReference(absent.state));
    EXPECT_TRUE(std::isnan(absent.candidate_length));
    EXPECT_EQ(absent.candidate_step, -1);
    EXPECT_EQ(absent.candidate_interval, -1);
    const auto retained = centering::update({5.0}, {{interval}});
    EXPECT_DOUBLE_EQ(retained.state.reference_length, 5.0);
  }
}

TEST(IntervalCenteringState, AmbiguousContainmentIsExcludedBeforeReliabilityFiltering)
{
  for (int condition = 0; condition < 3; ++condition) {
    auto overlap = observed(0.5);
    if (condition == 1) {overlap.nominal_fallback = true;}
    if (condition == 2) {overlap.start_boundary_observed = false;}
    const Preview preview{{observed(1.0), overlap}, {observed(4.8)}};
    const auto info = centering::update({}, preview);
    EXPECT_DOUBLE_EQ(info.state.reference_length, 4.8);
    EXPECT_EQ(info.candidate_step, 1);
    EXPECT_FALSE(centering::hasReference(centering::update({}, {preview[0]}).state));
  }
}

TEST(IntervalCenteringState, MissingCandidateRetainsB)
{
  const auto info = centering::update({4.8}, {{}, {}});
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.8);
  EXPECT_TRUE(std::isnan(info.candidate_length));
  EXPECT_EQ(info.candidate_step, -1);
}

TEST(IntervalCenteringState, ExcessiveIncreaseRetainsBAndReportsRejectedCandidate)
{
  auto info = centering::update({4.0}, {{observed(6.01)}});
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
  EXPECT_DOUBLE_EQ(info.candidate_length, 6.01);
  EXPECT_EQ(info.candidate_step, 0);
  EXPECT_EQ(info.candidate_interval, 0);
  info = centering::update(info.state, {{observed(10.0)}});
  EXPECT_DOUBLE_EQ(info.state.reference_length, 4.0);
}

TEST(IntervalCenteringState, InclusiveIncreaseAndImmediateDecreaseNeedNoHistory)
{
  const auto increased = centering::update({4.0}, {{observed(6.0)}});
  EXPECT_DOUBLE_EQ(increased.state.reference_length, 6.0);
  const auto decreased = centering::update(increased.state, {{observed(1.0)}});
  EXPECT_DOUBLE_EQ(decreased.state.reference_length, 1.0);
}

TEST(IntervalCenteringState, InvalidBInitializesFromCandidate)
{
  for (const double invalid : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    EXPECT_DOUBLE_EQ(centering::update({invalid}, {{observed(4.8)}}).state.reference_length, 4.8);
  }
}

TEST(IntervalCenteringState, OnOffUsesInclusiveRelativeThresholdAndFixedContraction)
{
  const auto on = centering::treatment(observed(6.0), {4.0}, {}, 0.7, 2.0, 9.0);
  EXPECT_TRUE(on.centering_on);
  EXPECT_DOUBLE_EQ(on.threshold, 6.0);
  EXPECT_NEAR(6.0 - 2.0 * on.inset, 2.4, 1e-12);
  const auto off = centering::treatment(observed(6.01), {4.0}, {}, 0.7, 2.0, 9.0);
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
}

TEST(IntervalCenteringState, TreatmentDoesNotRequireCandidateEligibility)
{
  auto clipped = observed(4.0, false);
  clipped.start_boundary_observed = false;
  const auto on = centering::treatment(clipped, {4.0}, {}, 0.7, 2.0, 9.0);
  EXPECT_TRUE(on.centering_on);
  EXPECT_NEAR(on.inset, 1.2, 1e-12);
}

TEST(IntervalCenteringState, FallbackHasIndependentNineMetreThreshold)
{
  auto fallback = observed(4.5);
  fallback.nominal_fallback = true;
  for (const centering::State state : {centering::State{}, centering::State{1.0}}) {
    const auto decision = centering::treatment(fallback, state, {}, 0.7, 2.0, 9.0);
    EXPECT_TRUE(decision.centering_on);
    EXPECT_DOUBLE_EQ(decision.threshold, 9.0);
    EXPECT_NEAR(4.5 - 2.0 * decision.inset, 1.8, 1e-12);
  }
  fallback.length = 9.0;
  EXPECT_TRUE(centering::treatment(fallback, {}, {}, 0.7, 2.0, 9.0).centering_on);
  fallback.length = 9.01;
  EXPECT_FALSE(centering::treatment(fallback, {}, {}, 0.7, 2.0, 9.0).centering_on);
  EXPECT_FALSE(centering::treatment(observed(4.5), {1.0}, {}, 0.7, 2.0, 9.0).centering_on);
}
