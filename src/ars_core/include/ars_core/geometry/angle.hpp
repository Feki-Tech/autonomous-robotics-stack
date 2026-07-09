#pragma once

#include <cmath>
#include <concepts>
#include <numbers>

namespace ars::core::geometry {

/// An angle normalized to the half-open interval [-pi, pi).
///
/// Arithmetic on raw radians is a classic source of robotics bugs (wrap-around
/// at +/-pi). Angle keeps every value normalized by construction, so
/// composition and differencing are always well-defined.
template <std::floating_point T = double>
class Angle {
 public:
  constexpr Angle() = default;

  /// Constructs from radians, normalizing into [-pi, pi).
  constexpr explicit Angle(T radians) : radians_{normalize(radians)} {}

  [[nodiscard]] static constexpr Angle from_degrees(T degrees) {
    return Angle{degrees * std::numbers::pi_v<T> / T{180}};
  }

  [[nodiscard]] constexpr T radians() const { return radians_; }
  [[nodiscard]] constexpr T degrees() const { return radians_ * T{180} / std::numbers::pi_v<T>; }

  constexpr Angle operator+(Angle rhs) const { return Angle{radians_ + rhs.radians_}; }
  constexpr Angle operator-(Angle rhs) const { return Angle{radians_ - rhs.radians_}; }
  constexpr Angle operator-() const { return Angle{-radians_}; }

  /// Shortest signed distance from rhs to this angle, in [-pi, pi).
  [[nodiscard]] constexpr T shortest_distance_to(Angle target) const {
    return (target - *this).radians();
  }

  [[nodiscard]] T sin() const { return std::sin(radians_); }
  [[nodiscard]] T cos() const { return std::cos(radians_); }

  constexpr bool operator==(const Angle&) const = default;

 private:
  /// Maps any finite value into [-pi, pi).
  static constexpr T normalize(T radians) {
    constexpr T two_pi = T{2} * std::numbers::pi_v<T>;
    T r = radians - two_pi * static_cast<T>(static_cast<long long>(radians / two_pi));
    if (r >= std::numbers::pi_v<T>) {
      r -= two_pi;
    } else if (r < -std::numbers::pi_v<T>) {
      r += two_pi;
    }
    return r;
  }

  T radians_{};
};

}  // namespace ars::core::geometry
