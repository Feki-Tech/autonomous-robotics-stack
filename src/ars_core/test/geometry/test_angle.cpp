#include <gtest/gtest.h>

#include <numbers>

#include "ars_core/geometry/angle.hpp"

namespace ars::core::geometry {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

TEST(AngleTest, DefaultConstructsToZero) {
  constexpr Angle a;
  EXPECT_DOUBLE_EQ(a.radians(), 0.0);
}

TEST(AngleTest, StoresValuesInsideRangeUnchanged) {
  constexpr Angle a{1.0};
  EXPECT_DOUBLE_EQ(a.radians(), 1.0);
}

TEST(AngleTest, NormalizesAbovePi) {
  const Angle a{kPi + 0.5};
  EXPECT_NEAR(a.radians(), -kPi + 0.5, kTol);
}

TEST(AngleTest, NormalizesBelowMinusPi) {
  const Angle a{-kPi - 0.5};
  EXPECT_NEAR(a.radians(), kPi - 0.5, kTol);
}

TEST(AngleTest, NormalizesMultipleWraps) {
  const Angle a{4.0 * kPi + 0.25};
  EXPECT_NEAR(a.radians(), 0.25, kTol);
  const Angle b{-6.0 * kPi - 0.25};
  EXPECT_NEAR(b.radians(), -0.25, kTol);
}

TEST(AngleTest, PiMapsToMinusPi) {
  const Angle a{kPi};
  EXPECT_NEAR(a.radians(), -kPi, kTol);
}

TEST(AngleTest, DegreesRoundTrip) {
  const auto a = Angle<>::from_degrees(90.0);
  EXPECT_NEAR(a.radians(), kPi / 2.0, kTol);
  EXPECT_NEAR(a.degrees(), 90.0, kTol);
}

TEST(AngleTest, AdditionWraps) {
  const Angle a{3.0};
  const Angle b{1.0};
  EXPECT_NEAR((a + b).radians(), 4.0 - 2.0 * kPi, kTol);
}

TEST(AngleTest, SubtractionWraps) {
  const Angle a{-3.0};
  const Angle b{1.5};
  EXPECT_NEAR((a - b).radians(), -4.5 + 2.0 * kPi, kTol);
}

TEST(AngleTest, Negation) {
  const Angle a{1.25};
  EXPECT_NEAR((-a).radians(), -1.25, kTol);
}

TEST(AngleTest, ShortestDistanceCrossesSeam) {
  const Angle from{kPi - 0.1};
  const Angle to{-kPi + 0.1};
  EXPECT_NEAR(from.shortest_distance_to(to), 0.2, kTol);
  EXPECT_NEAR(to.shortest_distance_to(from), -0.2, kTol);
}

TEST(AngleTest, TrigonometricAccessors) {
  const Angle a{kPi / 6.0};
  EXPECT_NEAR(a.sin(), 0.5, kTol);
  EXPECT_NEAR(a.cos(), std::sqrt(3.0) / 2.0, kTol);
}

}  // namespace
}  // namespace ars::core::geometry
