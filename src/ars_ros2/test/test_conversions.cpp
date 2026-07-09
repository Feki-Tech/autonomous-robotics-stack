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

TEST(ConversionsTest, OccupancyGridConversionPreservesGeometry) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 4;
  msg.info.height = 3;
  msg.info.resolution = 0.5;
  msg.info.origin.position.x = -1.0;
  msg.info.origin.position.y = 2.0;
  msg.data.assign(12, 0);

  const auto grid = to_grid(msg);
  EXPECT_EQ(grid.width(), 4U);
  EXPECT_EQ(grid.height(), 3U);
  EXPECT_DOUBLE_EQ(grid.resolution(), 0.5);
  EXPECT_DOUBLE_EQ(grid.origin().x, -1.0);
  EXPECT_DOUBLE_EQ(grid.origin().y, 2.0);
}

TEST(ConversionsTest, OccupancyGridTreatsUnknownAndOccupiedAsBlocked) {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.width = 3;
  msg.info.height = 1;
  msg.info.resolution = 1.0;
  msg.data = {0, -1, 100};  // free, unknown, occupied

  const auto grid = to_grid(msg);
  EXPECT_TRUE(grid.is_free({0, 0}));
  EXPECT_FALSE(grid.is_free({1, 0}));
  EXPECT_FALSE(grid.is_free({2, 0}));
}

TEST(ConversionsTest, PathRoundTrip) {
  const std::vector<core::geometry::Vec2<double>> points{{0.0, 0.0}, {1.0, 0.5}, {2.0, -1.0}};
  std_msgs::msg::Header header;
  header.frame_id = "odom";

  const auto msg = to_path_msg(points, header);
  ASSERT_EQ(msg.poses.size(), points.size());
  EXPECT_EQ(msg.header.frame_id, "odom");
  EXPECT_EQ(msg.poses.front().header.frame_id, "odom");
  EXPECT_DOUBLE_EQ(msg.poses[1].pose.orientation.w, 1.0);

  const auto restored = from_msg(msg);
  ASSERT_EQ(restored.size(), points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_DOUBLE_EQ(restored[i].x, points[i].x);
    EXPECT_DOUBLE_EQ(restored[i].y, points[i].y);
  }
}

}  // namespace
}  // namespace ars::ros2::conversions
