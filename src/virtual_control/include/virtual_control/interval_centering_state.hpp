#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace imac_ctrl::interval_centering
{

struct Settings
{
  double relative_length_factor{1.5};
  double scale_agreement_ratio{1.2};
  int scale_confirm_steps{3};
  double scale_max_relative_change{0.05};
};

struct Observation
{
  double length{0.0};
  bool start_boundary_observed{false};
  bool end_boundary_observed{false};
  bool reference_inside{false};
  bool nominal_fallback{false};
};

struct State
{
  // An observed interval-length scale, not an estimate of physical road width.
  double reference_length{0.0};
  std::vector<double> confirmation_lengths;
};

enum class UpdateReason : int
{
  NoCandidate = 0,
  Initialized = 1,
  MultipleObservedIntervals = 2,
  Confirming = 3,
  CandidatesDisagree = 4,
  Updated = 5
};

inline const char * reasonName(UpdateReason reason)
{
  switch (reason) {
    case UpdateReason::Initialized: return "initialized";
    case UpdateReason::MultipleObservedIntervals: return "multiple-observed-intervals";
    case UpdateReason::Confirming: return "confirming";
    case UpdateReason::CandidatesDisagree: return "candidates-disagree";
    case UpdateReason::Updated: return "updated";
    default: return "no-candidate";
  }
}

inline bool hasReference(const State & state)
{
  return std::isfinite(state.reference_length) && state.reference_length > 0.0;
}

struct UpdateInfo
{
  State state;
  double previous_reference_length{0.0};
  double candidate_length{std::numeric_limits<double>::quiet_NaN()};
  double update_target{std::numeric_limits<double>::quiet_NaN()};
  int candidate_first_step{-1};
  bool multiple_observed_intervals{false};
  UpdateReason reason{UpdateReason::NoCandidate};
};

inline double median(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  return values.size() % 2U == 0U ?
         0.5 * values[middle - 1U] + 0.5 * values[middle] : values[middle];
}

inline bool reliableSingleInterval(const std::vector<Observation> & intervals)
{
  if (intervals.size() != 1U) {return false;}
  const auto & interval = intervals.front();
  return !interval.nominal_fallback && interval.start_boundary_observed &&
         interval.end_boundary_observed && interval.reference_inside &&
         std::isfinite(interval.length) && interval.length > 0.0;
}

// Pure explicit state transition; called once after extracting the ENTIRE preview.
inline UpdateInfo update(
  const State & previous,
  const std::vector<std::vector<Observation>> & preview,
  const Settings & settings = Settings{})
{
  UpdateInfo info;
  info.state = previous;
  info.previous_reference_length = previous.reference_length;
  for (const auto & intervals : preview) {
    std::size_t observed_count = 0U;
    for (const auto & interval : intervals) {
      if (!interval.nominal_fallback && std::isfinite(interval.length) && interval.length > 0.0) {
        ++observed_count;
      }
    }
    info.multiple_observed_intervals |= observed_count > 1U;
  }

  // Search (0,1) before (1,2); never bridge a missing or unreliable node.
  const std::size_t near_count = std::min<std::size_t>(3U, preview.size());
  for (std::size_t k = 0; k + 1U < near_count; ++k) {
    if (!reliableSingleInterval(preview[k]) || !reliableSingleInterval(preview[k + 1U])) {
      continue;
    }
    const double first = preview[k].front().length;
    const double second = preview[k + 1U].front().length;
    if (std::max(first, second) / std::min(first, second) <= settings.scale_agreement_ratio) {
      info.candidate_first_step = static_cast<int>(k);
      info.candidate_length = 0.5 * first + 0.5 * second;
      break;
    }
  }

  if (!hasReference(previous)) {
    info.state = State{};
    if (info.candidate_first_step >= 0) {
      info.state.reference_length = info.candidate_length;
      info.reason = UpdateReason::Initialized;
    }
    return info;
  }
  if (info.multiple_observed_intervals) {
    info.state.confirmation_lengths.clear();
    info.reason = UpdateReason::MultipleObservedIntervals;
    return info;
  }
  if (info.candidate_first_step < 0) {
    info.state.confirmation_lengths.clear();
    return info;
  }

  // Rolling window of the most recent consecutive valid candidates. Only a
  // missing near candidate or an observed branch clears this confirmation history.
  auto & history = info.state.confirmation_lengths;
  history.push_back(info.candidate_length);
  const std::size_t count = static_cast<std::size_t>(settings.scale_confirm_steps);
  if (history.size() > count) {
    history.erase(history.begin(), history.end() - count);
  }
  if (history.size() < count) {
    info.reason = UpdateReason::Confirming;
    return info;
  }
  const auto bounds = std::minmax_element(history.begin(), history.end());
  if (*bounds.second / *bounds.first > settings.scale_agreement_ratio) {
    info.reason = UpdateReason::CandidatesDisagree;
    return info;
  }
  info.update_target = median(history);
  const double factor = 1.0 + settings.scale_max_relative_change;
  info.state.reference_length = std::clamp(
    info.update_target, previous.reference_length / factor,
    previous.reference_length * factor);
  info.reason = UpdateReason::Updated;
  return info;
}

struct Treatment
{
  bool centering_on{false};
  double threshold{std::numeric_limits<double>::quiet_NaN()};
  double inset{0.0};
};

inline Treatment treatment(
  const Observation & interval, const State & state, const Settings & settings,
  double soft_ratio, double boundary_margin, double fallback_max_centering_length)
{
  Treatment result;
  if (interval.nominal_fallback) {
    result.threshold = fallback_max_centering_length;
  } else if (hasReference(state)) {
    result.threshold = settings.relative_length_factor * state.reference_length;
  }
  result.centering_on = std::isfinite(result.threshold) && interval.length <= result.threshold;
  result.inset = result.centering_on ? (1.0 - soft_ratio) * interval.length :
    std::min(boundary_margin, 0.45 * interval.length);
  return result;
}

}  // namespace imac_ctrl::interval_centering
