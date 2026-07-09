#include <gtest/gtest.h>

#include <numbers>

#include "ars_ros2/conversions.hpp"

namespace ars::ros2::conversions {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

TEST(ConversionsTest, Pose2RoundTrip) {
  const core::geometry::Pose2<double> pose{1.5, -2.5, 0.75};
  const auto restored = from_msg(to_msg(pose));
  EXPECT_NEAR(restored.x(), pose.x(), kTol);
  EXPECT_NEAR(restored.y(), pose.y(), kTol);
  EXPECT_NEAR(restored.rotation().radians(), pose.rotation().radians(), kTol);
}

TEST(ConversionsTest, Pose2QuaternionIsNormalized) {
  const auto msg = to_msg(core::geometry::Pose2<double>{0.0, 0.0, 2.0});
  const double norm = msg.orientation.x * msg.orientation.x +
                      msg.orientation.y * msg.orientation.y +
                      msg.orientation.z * msg.orientation.z + msg.orientation.w * msg.orientation.w;
  EXPECT_NEAR(norm, 1.0, kTol);
  EXPECT_DOUBLE_EQ(msg.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.orientation.y, 0.0);
  EXPECT_DOUBLE_EQ(msg.position.z, 0.0);
}

TEST(ConversionsTest, Pose2YawNearPiSurvivesRoundTrip) {
  const core::geometry::Pose2<double> pose{0.0, 0.0, kPi - 1e-9};
  const auto restored = from_msg(to_msg(pose));
  EXPECT_NEAR(restored.rotation().radians(), kPi - 1e-9, 1e-9);
}

TEST(ConversionsTest, Twist2RoundTrip) {
  const core::geometry::Twist2<double> twist{0.7, -0.1, 1.4};
  const auto restored = from_msg(to_msg(twist));
  EXPECT_DOUBLE_EQ(restored.linear.x, twist.linear.x);
  EXPECT_DOUBLE_EQ(restored.linear.y, twist.linear.y);
  EXPECT_DOUBLE_EQ(restored.angular, twist.angular);
}

TEST(ConversionsTest, Twist2OutOfPlaneComponentsAreZero) {
  const auto msg = to_msg(core::geometry::Twist2<double>{1.0, 2.0, 3.0});
  EXPECT_DOUBLE_EQ(msg.linear.z, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(msg.angular.y, 0.0);
}

}  // namespace
}  // namespace ars::ros2::conversions
