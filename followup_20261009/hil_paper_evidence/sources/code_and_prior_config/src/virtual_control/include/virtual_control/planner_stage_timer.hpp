#pragma once

#include <chrono>
#include <limits>
#include <time.h>
#include <vector>

namespace imac_ctrl
{

// Elapsed wall time includes descheduling and blocking; thread CPU excludes both.
// Neither measurement alone identifies the resource responsible for a wait.
class PlannerStageTimer
{
public:
  struct Stage
  {
    const char * name;
    double wall_ms;
    double cpu_ms;
  };

  explicit PlannerStageTimer(std::chrono::steady_clock::time_point start =
    std::chrono::steady_clock::now())
  : start_(start), previous_(start), start_cpu_ms_(threadCpuMs()), previous_cpu_ms_(start_cpu_ms_)
  {
    stages_.reserve(24);
  }

  void mark(const char * name)
  {
    const auto now = std::chrono::steady_clock::now();
    const double cpu_ms = threadCpuMs();
    stages_.push_back({name,
      std::chrono::duration<double, std::milli>(now - previous_).count(),
      cpu_ms - previous_cpu_ms_});
    previous_ = now;
    previous_cpu_ms_ = cpu_ms;
  }

  double wallMs() const
  {
    return std::chrono::duration<double, std::milli>(previous_ - start_).count();
  }
  double cpuMs() const {return previous_cpu_ms_ - start_cpu_ms_;}
  const std::vector<Stage> & stages() const {return stages_;}

private:
  static double threadCpuMs()
  {
    timespec value{};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    return 1000.0 * value.tv_sec + 1e-6 * value.tv_nsec;
  }
  std::chrono::steady_clock::time_point start_;
  std::chrono::steady_clock::time_point previous_;
  double start_cpu_ms_;
  double previous_cpu_ms_;
  std::vector<Stage> stages_;
};

}  // namespace imac_ctrl
