#pragma once

#include <algorithm>
#include <concepts>
#include <optional>

namespace ars::core::control {

/// Configuration for a PID controller.
template <std::floating_point T = double>
struct PidConfig {
  T kp{};
  T ki{};
  T kd{};
  /// Symmetric output saturation; disabled when nullopt.
  std::optional<T> output_limit{};
  /// Symmetric clamp on the integral term (anti-windup); disabled when nullopt.
  std::optional<T> integral_limit{};
  /// First-order low-pass time constant for the derivative term [s].
  /// Zero disables filtering.
  T derivative_filter_tau{};
};

/// Discrete PID controller with anti-windup and filtered derivative.
///
/// Two windup defenses: an explicit integral clamp, and conditional
/// integration (the integral freezes while the output saturates in the same
/// direction as the error). The derivative acts on the measurement, not the
/// error, so setpoint steps do not produce derivative kick; an optional
/// first-order filter suppresses measurement noise amplification.
template <std::floating_point T = double>
class PidController {
 public:
  constexpr explicit PidController(PidConfig<T> config) : config_{config} {}

  [[nodiscard]] constexpr const PidConfig<T>& config() const { return config_; }

  constexpr void reset() {
    integral_ = T{};
    filtered_derivative_ = T{};
    previous_measurement_.reset();
  }

  /// Computes the control output for one timestep. dt must be positive.
  constexpr T update(T setpoint, T measurement, T dt) {
    const T error = setpoint - measurement;

    // Derivative on measurement (sign flipped) to avoid setpoint kick.
    T derivative{};
    if (previous_measurement_.has_value()) {
      const T raw = -(measurement - *previous_measurement_) / dt;
      if (config_.derivative_filter_tau > T{}) {
        const T alpha = dt / (config_.derivative_filter_tau + dt);
        filtered_derivative_ += alpha * (raw - filtered_derivative_);
        derivative = filtered_derivative_;
      } else {
        derivative = raw;
        filtered_derivative_ = raw;
      }
    }
    previous_measurement_ = measurement;

    const T candidate_integral = clamp_integral(integral_ + error * dt);
    T output = config_.kp * error + config_.ki * candidate_integral + config_.kd * derivative;

    if (config_.output_limit.has_value()) {
      const T limit = *config_.output_limit;
      const T saturated = std::clamp(output, -limit, limit);
      // Conditional integration: only accept the new integral if the output
      // is unsaturated or the error drives the output away from the limit.
      const bool saturating_further =
          (output > saturated && error > T{}) || (output < saturated && error < T{});
      if (!saturating_further) {
        integral_ = candidate_integral;
      }
      return saturated;
    }
    integral_ = candidate_integral;
    return output;
  }

 private:
  constexpr T clamp_integral(T value) const {
    if (config_.integral_limit.has_value()) {
      return std::clamp(value, -*config_.integral_limit, *config_.integral_limit);
    }
    return value;
  }

  PidConfig<T> config_;
  T integral_{};
  T filtered_derivative_{};
  std::optional<T> previous_measurement_{};
};

}  // namespace ars::core::control
