#pragma once

#include <cmath>
#include <concepts>

namespace ars::core::geometry {

/// 2D vector with basic arithmetic and geometric operations.
template <std::floating_point T = double>
struct Vec2 {
  T x{};
  T y{};

  constexpr Vec2() = default;
  constexpr Vec2(T x, T y) : x{x}, y{y} {}

  constexpr Vec2 operator+(const Vec2& rhs) const { return {x + rhs.x, y + rhs.y}; }
  constexpr Vec2 operator-(const Vec2& rhs) const { return {x - rhs.x, y - rhs.y}; }
  constexpr Vec2 operator*(T scalar) const { return {x * scalar, y * scalar}; }

  constexpr T dot(const Vec2& rhs) const { return x * rhs.x + y * rhs.y; }
  constexpr T cross(const Vec2& rhs) const { return x * rhs.y - y * rhs.x; }
  constexpr T norm_squared() const { return dot(*this); }

  T norm() const { return std::sqrt(norm_squared()); }

  Vec2 normalized() const {
    const auto n = norm();
    return {x / n, y / n};
  }

  constexpr bool operator==(const Vec2&) const = default;
};

}  // namespace ars::core::geometry
