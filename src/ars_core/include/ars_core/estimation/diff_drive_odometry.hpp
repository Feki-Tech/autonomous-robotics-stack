#pragma once

#include <concepts>

#include "ars_core/geometry/pose2.hpp"
#include "ars_core/geometry/twist2.hpp"

namespace ars::core::estimation {

/// Kinematic parameters of a differential-drive base.
template <std::floating_point T = double>
struct DiffDriveModel {
  T wheel_radius{};      ///< [m]
  T wheel_separation{};  ///< distance between wheel contact points [m]

  /// Forward kinematics: wheel angular rates [rad/s] -> body-frame twist.
  [[nodiscard]] constexpr geometry::Twist2<T> forward(T left_rate, T right_rate) const {
    const T v_left = left_rate * wheel_radius;
    const T v_right = right_rate * wheel_radius;
    return {(v_left + v_right) / T{2}, T{0}, (v_right - v_left) / wheel_separation};
  }
};

/// Dead-reckoning odometry for a differential-drive robot.
///
/// Integrates wheel encoder rates into a pose estimate using exact arc
/// integration (constant-twist motion between samples), which stays accurate
/// through tight turns where Euler integration drifts.
template <std::floating_point T = double>
class DiffDriveOdometry {
 public:
  constexpr DiffDriveOdometry(DiffDriveModel<T> model, geometry::Pose2<T> initial_pose = {})
      : model_{model}, pose_{initial_pose} {}

  [[nodiscard]] constexpr const geometry::Pose2<T>& pose() const { return pose_; }
  [[nodiscard]] constexpr const geometry::Twist2<T>& velocity() const { return velocity_; }

  constexpr void reset(const geometry::Pose2<T>& pose) {
    pose_ = pose;
    velocity_ = {};
  }

  /// Advances the estimate by dt seconds of the given wheel rates [rad/s].
  void update(T left_rate, T right_rate, T dt) {
    velocity_ = model_.forward(left_rate, right_rate);
    const T v = velocity_.linear.x;
    const T w = velocity_.angular;

    geometry::Pose2<T> delta{};
    if (is_straight(w, v)) {
      delta = {v * dt, T{0}, w * dt};
    } else {
      // Exact integration along a circular arc of radius v/w.
      const T radius = v / w;
      const T dtheta = w * dt;
      delta = {radius * std::sin(dtheta), radius * (T{1} - std::cos(dtheta)), dtheta};
    }
    pose_ = pose_.compose(delta);
  }

 private:
  static bool is_straight(T w, T v) {
    // Below this yaw rate the arc model degenerates numerically.
    constexpr T kMinCurvatureRate = T{1e-9};
    return std::abs(w) < kMinCurvatureRate * std::max(T{1}, std::abs(v));
  }

  DiffDriveModel<T> model_;
  geometry::Pose2<T> pose_;
  geometry::Twist2<T> velocity_{};
};

}  // namespace ars::core::estimation
