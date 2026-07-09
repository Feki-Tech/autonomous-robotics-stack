#include <gtest/gtest.h>

#include <cmath>

#include "ars_core/geometry/vec2.hpp"

namespace ars::core::geometry {
namespace {

TEST(Vec2Test, DefaultConstructsToZero) {
  constexpr Vec2 v;
  EXPECT_DOUBLE_EQ(v.x, 0.0);
  EXPECT_DOUBLE_EQ(v.y, 0.0);
}

TEST(Vec2Test, Addition) {
  constexpr Vec2 a{1.0, 2.0};
  constexpr Vec2 b{3.0, 4.0};
  constexpr auto c = a + b;
  EXPECT_DOUBLE_EQ(c.x, 4.0);
  EXPECT_DOUBLE_EQ(c.y, 6.0);
}

TEST(Vec2Test, Subtraction) {
  constexpr Vec2 a{5.0, 7.0};
  constexpr Vec2 b{2.0, 3.0};
  constexpr auto c = a - b;
  EXPECT_DOUBLE_EQ(c.x, 3.0);
  EXPECT_DOUBLE_EQ(c.y, 4.0);
}

TEST(Vec2Test, ScalarMultiplication) {
  constexpr Vec2 v{2.0, 3.0};
  constexpr auto scaled = v * 2.0;
  EXPECT_DOUBLE_EQ(scaled.x, 4.0);
  EXPECT_DOUBLE_EQ(scaled.y, 6.0);
}

TEST(Vec2Test, DotProduct) {
  constexpr Vec2 a{1.0, 0.0};
  constexpr Vec2 b{0.0, 1.0};
  EXPECT_DOUBLE_EQ(a.dot(b), 0.0);
  EXPECT_DOUBLE_EQ(a.dot(a), 1.0);
}

TEST(Vec2Test, CrossProduct) {
  constexpr Vec2 a{1.0, 0.0};
  constexpr Vec2 b{0.0, 1.0};
  EXPECT_DOUBLE_EQ(a.cross(b), 1.0);
  EXPECT_DOUBLE_EQ(b.cross(a), -1.0);
}

TEST(Vec2Test, Norm) {
  const Vec2 v{3.0, 4.0};
  EXPECT_DOUBLE_EQ(v.norm(), 5.0);
}

TEST(Vec2Test, Normalized) {
  const Vec2 v{3.0, 4.0};
  const auto unit = v.normalized();
  EXPECT_NEAR(unit.norm(), 1.0, 1e-12);
  EXPECT_NEAR(unit.x, 0.6, 1e-12);
  EXPECT_NEAR(unit.y, 0.8, 1e-12);
}

}  // namespace
}  // namespace ars::core::geometry
