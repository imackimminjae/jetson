#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace imac_ctrl::lower_steering_actuator_model
{

struct DiscreteModel
{
  double state_decay{0.0};
  double command_gain{1.0};
  double steering_integral_state_sec{0.0};
  double steering_integral_command_sec{0.0};
  int delay_steps{0};
};

// Exact zero-order-hold discretization of
//   delta_dot = (delta_command_delayed - delta) / tau.
// The integral coefficients are used by the bicycle yaw equation over one
// prediction interval, avoiding an additional Euler approximation.
inline DiscreteModel discretize(double dt_sec, double time_constant_sec, double delay_sec)
{
  if (!std::isfinite(dt_sec) || dt_sec <= 0.0) {
    throw std::invalid_argument("steering actuator dt must be finite and > 0");
  }
  if (!std::isfinite(time_constant_sec) || time_constant_sec <= 0.0) {
    throw std::invalid_argument("steering actuator time constant must be finite and > 0");
  }
  if (!std::isfinite(delay_sec) || delay_sec < 0.0) {
    throw std::invalid_argument("steering actuator delay must be finite and >= 0");
  }

  DiscreteModel model;
  model.state_decay = std::exp(-dt_sec / time_constant_sec);
  model.command_gain = 1.0 - model.state_decay;
  model.steering_integral_state_sec =
    time_constant_sec * model.command_gain;
  model.steering_integral_command_sec =
    dt_sec - model.steering_integral_state_sec;
  model.delay_steps = std::max(0, static_cast<int>(std::lround(delay_sec / dt_sec)));
  return model;
}

inline double advance(
  double effective_angle_rad, double delayed_command_rad,
  const DiscreteModel & model)
{
  return model.state_decay * effective_angle_rad +
         model.command_gain * delayed_command_rad;
}

}  // namespace imac_ctrl::lower_steering_actuator_model
