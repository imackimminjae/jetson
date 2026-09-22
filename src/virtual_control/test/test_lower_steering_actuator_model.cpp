#include "virtual_control/lower_steering_actuator_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace actuator = imac_ctrl::lower_steering_actuator_model;

TEST(LowerSteeringActuatorModel, IdentifiedParametersDiscretizeToFourDelaySteps)
{
  const auto model = actuator::discretize(0.1, 0.85, 0.4);

  EXPECT_EQ(model.delay_steps, 4);
  EXPECT_NEAR(model.state_decay, std::exp(-0.1 / 0.85), 1e-12);
  EXPECT_NEAR(model.command_gain, 1.0 - model.state_decay, 1e-12);
  EXPECT_NEAR(
    model.steering_integral_state_sec + model.steering_integral_command_sec,
    0.1, 1e-12);
}

TEST(LowerSteeringActuatorModel, ConstantCommandConvergesToCommand)
{
  const auto model = actuator::discretize(0.1, 0.85, 0.4);
  double effective = 0.0;
  constexpr double command = 0.30;

  for (int step = 0; step < 100; ++step) {
    effective = actuator::advance(effective, command, model);
  }

  EXPECT_NEAR(effective, command, 3e-6);
}

TEST(LowerSteeringActuatorModel, RejectsInvalidParameters)
{
  EXPECT_THROW(actuator::discretize(0.0, 0.85, 0.4), std::invalid_argument);
  EXPECT_THROW(actuator::discretize(0.1, 0.0, 0.4), std::invalid_argument);
  EXPECT_THROW(actuator::discretize(0.1, 0.85, -0.1), std::invalid_argument);
}
