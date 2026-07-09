#include <gtest/gtest.h>

#include <cmath>

#include "ars_core/control/pid_controller.hpp"

namespace ars::core::control {
namespace {

constexpr double kTol = 1e-12;

TEST(PidControllerTest, ProportionalOnly) {
  PidController pid{PidConfig<double>{.kp = 2.0}};
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.1), 2.0, kTol);
  EXPECT_NEAR(pid.update(1.0, 0.5, 0.1), 1.0, kTol);
}

TEST(PidControllerTest, IntegralAccumulates) {
  PidController pid{PidConfig<double>{.ki = 1.0}};
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 0.5, kTol);
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 1.0, kTol);
}

TEST(PidControllerTest, DerivativeOnMeasurementAvoidsSetpointKick) {
  PidController pid{PidConfig<double>{.kd = 1.0}};
  pid.update(0.0, 0.0, 0.1);
  // Setpoint jumps; measurement unchanged -> derivative term must stay zero.
  EXPECT_NEAR(pid.update(10.0, 0.0, 0.1), 0.0, kTol);
  // Measurement rises -> negative derivative (damping).
  EXPECT_LT(pid.update(10.0, 1.0, 0.1), 0.0);
}

TEST(PidControllerTest, DerivativeFilterSmoothsSteps) {
  PidController raw{PidConfig<double>{.kd = 1.0}};
  PidController filtered{PidConfig<double>{.kd = 1.0, .derivative_filter_tau = 1.0}};
  raw.update(0.0, 0.0, 0.1);
  filtered.update(0.0, 0.0, 0.1);
  const double raw_response = raw.update(0.0, 1.0, 0.1);
  const double filtered_response = filtered.update(0.0, 1.0, 0.1);
  EXPECT_LT(std::abs(filtered_response), std::abs(raw_response));
  EXPECT_LT(raw_response, 0.0);
  EXPECT_LT(filtered_response, 0.0);
}

TEST(PidControllerTest, OutputSaturates) {
  PidController pid{PidConfig<double>{.kp = 100.0, .output_limit = 5.0}};
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.1), 5.0, kTol);
  EXPECT_NEAR(pid.update(-1.0, 0.0, 0.1), -5.0, kTol);
}

TEST(PidControllerTest, IntegralClampLimitsWindup) {
  PidController pid{PidConfig<double>{.ki = 1.0, .integral_limit = 2.0}};
  for (int i = 0; i < 100; ++i) {
    pid.update(10.0, 0.0, 1.0);
  }
  EXPECT_NEAR(pid.update(10.0, 0.0, 1.0), 2.0, kTol);
}

TEST(PidControllerTest, ConditionalIntegrationPreventsWindup) {
  PidController pid{PidConfig<double>{.ki = 1.0, .output_limit = 1.0}};
  // Saturate hard for a long time.
  for (int i = 0; i < 1000; ++i) {
    EXPECT_NEAR(pid.update(10.0, 0.0, 1.0), 1.0, kTol);
  }
  // On reversal the output must leave saturation quickly, not after
  // unwinding 1000 seconds of accumulated integral.
  const double reversed = pid.update(-10.0, 0.0, 1.0);
  EXPECT_NEAR(reversed, -1.0, kTol);
}

TEST(PidControllerTest, ResetClearsState) {
  PidController pid{PidConfig<double>{.ki = 1.0, .kd = 1.0}};
  pid.update(1.0, 0.5, 0.1);
  pid.update(1.0, 0.7, 0.1);
  pid.reset();
  // After reset: no integral, no previous measurement -> pure fresh response.
  EXPECT_NEAR(pid.update(1.0, 0.0, 0.5), 0.5, kTol);
}

TEST(PidControllerTest, RegulatesFirstOrderPlant) {
  // Plant: dx/dt = -x + u. PI control must drive x to the setpoint.
  PidController pid{PidConfig<double>{.kp = 4.0, .ki = 2.0}};
  const double dt = 0.01;
  double x = 0.0;
  for (int i = 0; i < 5000; ++i) {
    const double u = pid.update(1.0, x, dt);
    x += dt * (-x + u);
  }
  EXPECT_NEAR(x, 1.0, 1e-6);
}

}  // namespace
}  // namespace ars::core::control
