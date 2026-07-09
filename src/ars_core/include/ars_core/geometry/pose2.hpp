#pragma once

#include <concepts>

#include "ars_core/geometry/angle.hpp"
#include "ars_core/geometry/vec2.hpp"

namespace ars::core::geometry {

/// A rigid transform in SE(2): translation + heading.
///
/// Follows the convention T_parent_child: `compose` chains child frames,
/// `transform` maps a point from the child frame into the parent frame.
template <std::floating_point T = double>
class Pose2 {
 public:
  constexpr Pose2() = default;
  constexpr Pose2(Vec2<T> translation, Angle<T> rotation)
      : translation_{translation}, rotation_{rotation} {}
  constexpr Pose2(T x, T y, T theta) : translation_{x, y}, rotation_{theta} {}

  [[nodiscard]] constexpr const Vec2<T>& translation() const { return translation_; }
  [[nodiscard]] constexpr Angle<T> rotation() const { return rotation_; }
  [[nodiscard]] constexpr T x() const { return translation_.x; }
  [[nodiscard]] constexpr T y() const { return translation_.y; }

  /// Maps a point from this pose's (child) frame into the parent frame.
  [[nodiscard]] Vec2<T> transform(const Vec2<T>& point) const {
    const T c = rotation_.cos();
    const T s = rotation_.sin();
    return {c * point.x - s * point.y + translation_.x,
            s * point.x + c * point.y + translation_.y};
  }

  /// T_a_c = T_a_b.compose(T_b_c)
  [[nodiscard]] Pose2 compose(const Pose2& rhs) const {
    return {transform(rhs.translation_), rotation_ + rhs.rotation_};
  }

  /// Inverse transform: T_b_a = T_a_b.inverse()
  [[nodiscard]] Pose2 inverse() const {
    const T c = rotation_.cos();
    const T s = rotation_.sin();
    return {{-(c * translation_.x + s * translation_.y),
             -(-s * translation_.x + c * translation_.y)},
            -rotation_};
  }

  /// Expresses `other` relative to this pose: T_this_other.
  [[nodiscard]] Pose2 relative_to(const Pose2& other) const {
    return other.inverse().compose(*this);
  }

  constexpr bool operator==(const Pose2&) const = default;

 private:
  Vec2<T> translation_{};
  Angle<T> rotation_{};
};

}  // namespace ars::core::geometry
