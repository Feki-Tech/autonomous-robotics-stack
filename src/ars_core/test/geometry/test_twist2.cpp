#include <gtest/gtest.h>

#include <numbers>

#include "ars_core/geometry/twist2.hpp"

namespace ars::core::geometry {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

TEST(Twist2Test, DefaultConstructsToZero) {
  constexpr Twist2 t;
  EXPECT_DOUBLE_EQ(t.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(t.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(t.angular, 0.0);
}

TEST(Twist2Test, Arithmetic) {
  constexpr Twist2 a{1.0, 2.0, 0.5};
  constexpr Twist2 b{0.5, -1.0, 0.25};
  constexpr auto sum = a + b;
  EXPECT_DOUBLE_EQ(sum.linear.x, 1.5);
  EXPECT_DOUBLE_EQ(sum.linear.y, 1.0);
  EXPECT_DOUBLE_EQ(sum.angular, 0.75);

  constexpr auto diff = a - b;
  EXPECT_DOUBLE_EQ(diff.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(diff.linear.y, 3.0);
  EXPECT_DOUBLE_EQ(diff.angular, 0.25);

  constexpr auto scaled = a * 2.0;
  EXPECT_DOUBLE_EQ(scaled.linear.x, 2.0);
  EXPECT_DOUBLE_EQ(scaled.linear.y, 4.0);
  EXPECT_DOUBLE_EQ(scaled.angular, 1.0);
}

TEST(Twist2Test, ToWorldRotatesLinearPart) {
  const Twist2 body{1.0, 0.0, 0.3};
  const auto world = body.to_world(Angle{kPi / 2.0});
  EXPECT_NEAR(world.linear.x, 0.0, kTol);
  EXPECT_NEAR(world.linear.y, 1.0, kTol);
  EXPECT_DOUBLE_EQ(world.angular, 0.3);
}

TEST(Twist2Test, ToBodyInvertsToWorld) {
  const Twist2 body{0.7, -0.2, 0.9};
  const Angle heading{1.234};
  const auto round_trip = body.to_world(heading).to_body(heading);
  EXPECT_NEAR(round_trip.linear.x, body.linear.x, kTol);
  EXPECT_NEAR(round_trip.linear.y, body.linear.y, kTol);
  EXPECT_DOUBLE_EQ(round_trip.angular, body.angular);
}

TEST(Twist2Test, IntegratePureTranslation) {
  const Twist2 t{2.0, 0.0, 0.0};
  const auto next = t.integrate(Pose2{0.0, 0.0, 0.0}, 0.5);
  EXPECT_NEAR(next.x(), 1.0, kTol);
  EXPECT_NEAR(next.y(), 0.0, kTol);
  EXPECT_NEAR(next.rotation().radians(), 0.0, kTol);
}

TEST(Twist2Test, IntegratePureRotationWraps) {
  const Twist2 t{0.0, 0.0, 1.0};
  const auto next = t.integrate(Pose2{1.0, 1.0, kPi - 0.1}, 0.2);
  EXPECT_NEAR(next.x(), 1.0, kTol);
  EXPECT_NEAR(next.y(), 1.0, kTol);
  EXPECT_NEAR(next.rotation().radians(), -kPi + 0.1, kTol);
}

TEST(Twist2Test, IntegrateUsesHeadingForTranslation) {
  const Twist2 t{1.0, 0.0, 0.0};
  const auto next = t.integrate(Pose2{0.0, 0.0, kPi / 2.0}, 1.0);
  EXPECT_NEAR(next.x(), 0.0, kTol);
  EXPECT_NEAR(next.y(), 1.0, kTol);
}

}  // namespace
}  // namespace ars::core::geometry
