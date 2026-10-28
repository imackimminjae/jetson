#include "virtual_control/tracking_control.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace imac_ctrl
{
// Access the planner's private helpers without constructing or spinning a node.
struct IntervalCenteringTestAccess
{
  using Observation = SdMapUpperPlannerNode::IntervalObservation;
  using State = SdMapUpperPlannerNode::IntervalCenteringState;
  using Settings = SdMapUpperPlannerNode::IntervalCenteringSettings;

  static auto probes(
    const std::vector<Eigen::Vector2d> & points, const std::vector<double> & headings,
    int horizon, double spacing)
  {
    return SdMapUpperPlannerNode::buildIntervalBProbes(points, headings, horizon, spacing);
  }

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
    double soft_ratio, double boundary_margin, double fallback_max_centering_length,
    int prediction_step = -1, bool k1_disable_extra_centering = false)
  {
    return SdMapUpperPlannerNode::intervalTreatment(
      interval, state, settings, soft_ratio, boundary_margin, fallback_max_centering_length,
      prediction_step, k1_disable_extra_centering);
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

TEST(IntervalCenteringState, K1FreedomKeepsBoundaryInsetInRecordedWideFailure)
{
  const auto legacy = centering::treatment(observed(18.9), {13.8}, {}, .7, 2., 9., 1, false);
  const auto expanded = centering::treatment(observed(18.9), {13.8}, {}, .7, 2., 9., 1, true);
  EXPECT_NEAR(legacy.inset, 5.67, 1e-12);
  EXPECT_NEAR(expanded.inset, 2., 1e-12);
  EXPECT_NEAR(18.9 - 2 * expanded.inset, 14.9, 1e-12);
  EXPECT_DOUBLE_EQ(expanded.threshold, legacy.threshold);
}

TEST(IntervalCenteringState, K1FreedomNeverTightensNarrowRoad)
{
  const auto expanded = centering::treatment(observed(4.9), {4.9}, {}, .7, 2., 9., 1, true);
  EXPECT_NEAR(expanded.inset, 1.47, 1e-12);
}

TEST(IntervalCenteringState, K1FreedomPreservesOtherStagesAndDisabledBehavior)
{
  const auto legacy = centering::treatment(observed(18.9), {13.8}, {}, .7, 2., 9.);
  for (int k = 0; k <= 7; ++k) {
    const auto disabled = centering::treatment(observed(18.9), {13.8}, {}, .7, 2., 9., k, false);
    EXPECT_DOUBLE_EQ(disabled.inset, legacy.inset);
    if (k != 1) {
      EXPECT_DOUBLE_EQ(centering::treatment(observed(18.9), {13.8}, {}, .7, 2., 9., k, true).inset, legacy.inset);
    }
  }
}

TEST(IntervalCenteringState, K1FreedomOnlyExpandsAndStaysWithinObservedBoundaries)
{
  for (double length : {.2, 1., 4.9, 6.7, 10.2, 18.9, 30.}) {
    for (double B : {0., 2., 4.9, 13.8, 30.}) {
      const auto legacy = centering::treatment(observed(length), {B}, {}, .7, 2., 9.);
      const auto expanded = centering::treatment(observed(length), {B}, {}, .7, 2., 9., 1, true);
      EXPECT_LE(expanded.inset, legacy.inset);
      EXPECT_GE(expanded.inset, 0.);
      EXPECT_LT(2 * expanded.inset, length);
      if (std::isnan(legacy.threshold)) {
        EXPECT_TRUE(std::isnan(expanded.threshold));
      } else {
        EXPECT_DOUBLE_EQ(expanded.threshold, legacy.threshold);
      }
    }
  }
}

TEST(IntervalCenteringState, SmoothTransitionPreservesEndsAndInterpolatesMiddle)
{
  centering::Settings settings;
  settings.transition_relative_band = 0.25;
  const centering::State state{10.0};
  for (double length : {10.0, 12.5, 15.0, 20.0}) {
    const auto result = centering::treatment(observed(length), state, settings, 0.7, 2.0, 9.0);
    EXPECT_NEAR(result.inset, length <= 12.5 ? 0.3 * length : 2.0, 1e-12);
  }
  const auto middle = centering::treatment(observed(13.75), state, settings, 0.7, 2.0, 9.0);
  EXPECT_NEAR(middle.inset, 0.5 * (2.0 + 0.3 * 13.75), 1e-12);
}

TEST(IntervalCenteringState, SmoothTransitionRemovesTwentyMetreWidthJump)
{
  centering::Settings settings;
  settings.transition_relative_band = 0.25;
  const auto interval = observed(20.0);
  const double boundary = 20.0 / 1.5;
  const auto before = centering::treatment(interval, {boundary - 1e-6}, settings, 0.7, 2.0, 9.0);
  const auto after = centering::treatment(interval, {boundary + 1e-6}, settings, 0.7, 2.0, 9.0);
  EXPECT_NEAR(before.inset, after.inset, 1e-9);
  EXPECT_NEAR(after.inset, 2.0, 1e-9);
}

TEST(IntervalCenteringState, SmoothTransitionPreservesFallbackAndMissingReference)
{
  centering::Settings settings;
  settings.transition_relative_band = 0.25;
  auto fallback = observed(4.5);
  fallback.nominal_fallback = true;
  EXPECT_NEAR(centering::treatment(fallback, {3.0}, settings, 0.7, 2.0, 9.0).inset, 1.35, 1e-12);
  EXPECT_DOUBLE_EQ(centering::treatment(observed(20.0), {}, settings, 0.7, 2.0, 9.0).inset, 2.0);
}

TEST(IntervalCenteringState, SmoothTransitionKeepsBoundsInsideRawInterval)
{
  centering::Settings settings;
  settings.transition_relative_band = 0.25;
  for (double B : {0.3, 2.0, 10.0, 20.0}) {
    for (double ratio : {0.1, 1.0, 1.25, 1.3, 1.375, 1.49, 1.5, 2.0}) {
      const double length = B * ratio;
      const auto result = centering::treatment(observed(length), {B}, settings, 0.7, 2.0, 9.0);
      EXPECT_GE(result.inset, 0.0);
      EXPECT_LT(2.0 * result.inset, length);
    }
  }
}

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

TEST(IntervalCenteringState, ZeroGrowthCapPreservesLegacySequence)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.0;
  centering::State state{4.0};
  for (const auto & sample : std::vector<std::array<double, 2>>{
      {6.01, 4.0}, {6.0, 6.0}, {10.0, 6.0}, {1.0, 1.0}, {4.8, 1.0}})
  {
    state = centering::update(state, {{observed(sample[0])}}, settings).state;
    EXPECT_DOUBLE_EQ(state.reference_length, sample[1]);
  }
}

TEST(IntervalCenteringState, GrowthCapInitializesAndKeepsNarrowingImmediate)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.1;  // 1 m/s at configured 10 Hz.
  auto info = centering::update({}, {{observed(10.0)}}, settings);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 10.0);
  info = centering::update(info.state, {{observed(1.0)}}, settings);
  EXPECT_DOUBLE_EQ(info.state.reference_length, 1.0);
  info = centering::update(info.state, {{observed(10.0)}}, settings);
  EXPECT_DOUBLE_EQ(info.candidate_length, 10.0);
  EXPECT_NEAR(info.state.reference_length, 1.1, 1e-12);
  // A narrow spike cannot permanently lock out the restored wider road.
  for (int i = 0; i < 90; ++i) {
    info = centering::update(info.state, {{observed(10.0)}}, settings);
  }
  EXPECT_DOUBLE_EQ(info.state.reference_length, 10.0);
}

TEST(IntervalCenteringState, GrowthCapConvergesToPersistentWideningBeyondLegacyGate)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.1;
  centering::State state{4.0};
  for (int i = 0; i < 61; ++i) {
    const auto info = centering::update(state, {{observed(10.0)}}, settings);
    EXPECT_GE(info.state.reference_length, state.reference_length);
    EXPECT_LE(info.state.reference_length - state.reference_length, 0.1 + 1e-12);
    EXPECT_LE(info.state.reference_length, 10.0);
    state = info.state;
  }
  EXPECT_DOUBLE_EQ(state.reference_length, 10.0);
}

TEST(IntervalCenteringState, GrowthCapMissingEvidenceRetainsBAndEligibility)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.1;
  auto clipped = observed(20.0);
  clipped.end_boundary_observed = false;
  auto fallback = observed(20.0);
  fallback.nominal_fallback = true;
  const auto missing = centering::update({4.8}, {{}, {clipped}, {fallback}}, settings);
  EXPECT_DOUBLE_EQ(missing.state.reference_length, 4.8);
  EXPECT_EQ(missing.candidate_step, -1);
  // Other branches and ambiguously overlapping intervals still supply no B.
  const auto ambiguous = centering::update(
    {4.8}, {{observed(1.0), observed(2.0)}, {observed(0.5, false)}}, settings);
  EXPECT_DOUBLE_EQ(ambiguous.state.reference_length, 4.8);
  EXPECT_EQ(ambiguous.candidate_step, -1);
}

TEST(IntervalCenteringState, GrowthCapRejectsInvalidLimits)
{
  for (double invalid : {-0.1, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    centering::Settings settings;
    settings.max_reference_growth_per_update_m = invalid;
    EXPECT_THROW(centering::update({4.0}, {{observed(5.0)}}, settings), std::invalid_argument);
  }
}

TEST(IntervalCenteringState, GrowthCapPreventsRecordedLongIntervalForcedContraction)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.1;
  centering::State legacy{11.8};
  centering::State limited = legacy;
  // S1 baseline: support changes from near k1 to wider k3/k4 sections.
  for (double candidate : {14.2, 12.2, 14.2, 14.3, 14.2, 14.3}) {
    legacy = centering::update(legacy, {{observed(candidate)}}).state;
    limited = centering::update(limited, {{observed(candidate)}}, settings).state;
  }
  EXPECT_NEAR(legacy.reference_length, 14.3, 1e-12);
  EXPECT_NEAR(limited.reference_length, 12.4, 1e-12);
  const auto old_treatment = centering::treatment(observed(20.6), legacy, {}, 0.7, 2.0, 9.0);
  const auto new_treatment = centering::treatment(observed(20.6), limited, settings, 0.7, 2.0, 9.0);
  EXPECT_TRUE(old_treatment.centering_on);
  EXPECT_FALSE(new_treatment.centering_on);
  EXPECT_NEAR(old_treatment.inset, 6.18, 1e-12);
  EXPECT_DOUBLE_EQ(new_treatment.inset, 2.0);
  EXPECT_NEAR(new_treatment.threshold, 1.5 * limited.reference_length, 1e-12);
}

TEST(IntervalCenteringState, NarrowSpikeThenNarrowReturnRecoversFromTemporaryOff)
{
  centering::Settings settings;
  settings.max_reference_growth_per_update_m = 0.1;
  const auto spike = centering::update({4.8}, {{observed(1.0)}});
  centering::State legacy = spike.state;
  centering::State limited = spike.state;
  EXPECT_DOUBLE_EQ(legacy.reference_length, 1.0);
  int limited_off_updates = 0;
  bool eventually_on = false;
  for (int i = 0; i < 40; ++i) {
    legacy = centering::update(legacy, {{observed(4.8)}}).state;
    limited = centering::update(limited, {{observed(4.8)}}, settings).state;
    // This comparison applies specifically to spike->constant 4.8 m return;
    // the capped update need not dominate legacy B for arbitrary sequences.
    EXPECT_DOUBLE_EQ(legacy.reference_length, 1.0);
    EXPECT_GE(limited.reference_length, legacy.reference_length);
    const auto old_treatment = centering::treatment(observed(4.8), legacy, {}, 0.7, 2.0, 9.0);
    const auto new_treatment = centering::treatment(observed(4.8), limited, settings, 0.7, 2.0, 9.0);
    EXPECT_FALSE(old_treatment.centering_on);
    if (!new_treatment.centering_on) {
      ++limited_off_updates;
      EXPECT_DOUBLE_EQ(new_treatment.inset, old_treatment.inset);
    } else {
      eventually_on = true;
      // Narrow OFF contraction is stronger here, but legacy already remains
      // OFF forever. Recovery eventually widens the usable interval instead.
      EXPECT_LT(new_treatment.inset, old_treatment.inset);
    }
  }
  EXPECT_GT(limited_off_updates, 0);
  EXPECT_LT(limited_off_updates, 40);
  EXPECT_TRUE(eventually_on);
  EXPECT_DOUBLE_EQ(limited.reference_length, 4.8);
}

TEST(IntervalCenteringState, DenseBReferencesAreAdditionalAndKeepTheSameHorizon)
{
  const std::vector<Eigen::Vector2d> points{{0,0}, {3,0}, {6,0}};
  const auto probes = centering::probes(points, {0,0,0}, 2, 0.6);
  ASSERT_EQ(probes.size(), 8u);  // Existing points at 0,3,6 are not duplicated.
  for (const auto & p : probes) {
    EXPECT_GT(p.arc_length_m, 0.0);
    EXPECT_LT(p.arc_length_m, 6.0);
    EXPECT_NEAR(p.point_body.x(), p.arc_length_m, 1e-12);
    EXPECT_DOUBLE_EQ(p.point_body.y(), 0.0);
    EXPECT_NEAR(p.heading_rad, 0.0, 1e-12);
    EXPECT_NE(p.arc_length_m, 3.0);
  }
  EXPECT_TRUE(centering::probes(points, {0,0,0}, 2, 0.0).empty());
  EXPECT_TRUE(centering::probes(points, {0,0,0}, 2, 20.0).empty());
}

TEST(IntervalCenteringState, DenseBReferencesFollowArclengthAndWrapHeading)
{
  const double pi = std::acos(-1.0);
  const auto probes = centering::probes(
    {{0,0},{2,0},{2,2}}, {pi - 0.1, -pi + 0.1, -pi + 0.3}, 2, 1.0);
  ASSERT_EQ(probes.size(), 2u);
  EXPECT_NEAR(probes[0].point_body.x(), 1.0, 1e-12);
  EXPECT_NEAR(probes[0].heading_rad, pi, 1e-12);
  EXPECT_EQ(probes[0].preceding_step, 0);
  EXPECT_NEAR(probes[1].point_body.x(), 2.0, 1e-12);
  EXPECT_NEAR(probes[1].point_body.y(), 1.0, 1e-12);
  EXPECT_EQ(probes[1].preceding_step, 1);
  EXPECT_EQ(centering::probes({{0,0},{2,0},{2,2}}, {0,0,0}, 1, 1.0).size(), 1u);
}

TEST(IntervalCenteringState, DenseBReferencesRejectInvalidOrDegenerateInputs)
{
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(centering::probes({{0,0},{1,0}}, {0}, 1, 0.6).empty());
  EXPECT_TRUE(centering::probes({{0,0},{nan,0}}, {0,0}, 1, 0.6).empty());
  EXPECT_TRUE(centering::probes({{0,0},{1,0}}, {0,nan}, 1, 0.6).empty());
  EXPECT_TRUE(centering::probes({{0,0},{0,0}}, {0,0}, 1, 0.6).empty());
  EXPECT_TRUE(centering::probes({{0,0},{1,0}}, {0,0}, 1, nan).empty());
  EXPECT_TRUE(centering::probes({{0,0},{1,0}}, {0,0}, 0, 0.6).empty());
}
