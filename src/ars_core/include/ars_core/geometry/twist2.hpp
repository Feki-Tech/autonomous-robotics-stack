#pragma once

#include <concepts>

#include "ars_core/geometry/angle.hpp"
#include "ars_core/geometry/pose2.hpp"
#include "ars_core/geometry/vec2.hpp"

namespace ars::core::geometry {

/// A planar velocity: linear (m/s) and angular (rad/s) components.
///
/// Frame-agnostic by itself; `to_world`/`to_body` convert the linear part
/// between the body frame and the world frame given the robot heading.
/// Angular velocity is invariant under planar frame rotation.
template <std::floating_point T = double>
struct Twist2 {
  Vec2<T> linear{};
  T angular{};

  constexpr Twist2() = default;
  constexpr Twist2(Vec2<T> linear_velocity, T angular_velocity)
      : linear{linear_velocity}, angular{angular_velocity} {}
  constexpr Twist2(T vx, T vy, T wz) : linear{vx, vy}, angular{wz} {}

  constexpr Twist2 operator+(const Twist2& rhs) const {
    return {linear + rhs.linear, angular + rhs.angular};
  }
  constexpr Twist2 operator-(const Twist2& rhs) const {
    return {linear - rhs.linear, angular - rhs.angular};
  }
  constexpr Twist2 operator*(T scalar) const { return {linear * scalar, angular * scalar}; }

  /// Rotates the linear part from body frame into world frame.
  [[nodiscard]] Twist2 to_world(Angle<T> heading) const {
    const T c = heading.cos();
    const T s = heading.sin();
    return {{c * linear.x - s * linear.y, s * linear.x + c * linear.y}, angular};
  }

  /// Rotates the linear part from world frame into body frame.
  [[nodiscard]] Twist2 to_body(Angle<T> heading) const { return to_world(-heading); }

  /// First-order integration of this body-frame twist over dt seconds,
  /// applied to `pose` (exact for pure translation or pure rotation steps).
  [[nodiscard]] Pose2<T> integrate(const Pose2<T>& pose, T dt) const {
    const auto world = to_world(pose.rotation());
    return {pose.translation() + world.linear * dt, pose.rotation() + Angle<T>{angular * dt}};
  }

  constexpr bool operator==(const Twist2&) const = default;
};

}  // namespace ars::core::geometry
