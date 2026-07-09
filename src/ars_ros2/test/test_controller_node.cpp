#include <gtest/gtest.h>

#include <cmath>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ars_ros2/controller_node.hpp"

namespace {

using ars::ros2::ControllerNode;

class ControllerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static nav_msgs::msg::Path straight_path(double length) {
    nav_msgs::msg::Path path;
    for (double x = 0.0; x <= length; x += 0.5) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    return path;
  }

  static nav_msgs::msg::Odometry odom_at(double x, double y, double yaw = 0.0) {
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    return msg;
  }
};

TEST_F(ControllerNodeTest, ConfiguresWithDefaultParameters) {
  ControllerNode node;
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(ControllerNodeTest, RejectsInvalidConfiguration) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter{"cruise_speed", -1.0}});
  ControllerNode node{options};
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(ControllerNodeTest, ZeroCommandWithoutPath) {
  ControllerNode node;
  node.configure();
  const auto command = node.handle_odom(odom_at(0.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
}

TEST_F(ControllerNodeTest, DrivesStraightOnAlignedPath) {
  ControllerNode node;
  node.configure();
  node.handle_path(straight_path(5.0));
  const auto command = node.handle_odom(odom_at(0.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.5);
  EXPECT_NEAR(command.angular.z, 0.0, 1e-9);
}

TEST_F(ControllerNodeTest, SteersTowardPathWhenOffset) {
  ControllerNode node;
  node.configure();
  node.handle_path(straight_path(5.0));
  // Robot below the path (y < 0), heading along +x: must steer left (w > 0).
  const auto command = node.handle_odom(odom_at(1.0, -0.5));
  EXPECT_GT(command.angular.z, 0.0);
}

TEST_F(ControllerNodeTest, StopsAtGoal) {
  ControllerNode node;
  node.configure();
  node.handle_path(straight_path(5.0));
  const auto command = node.handle_odom(odom_at(5.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular.z, 0.0);
  EXPECT_TRUE(node.goal_reached());

  // Stays stopped on subsequent samples.
  const auto next = node.handle_odom(odom_at(5.0, 0.0));
  EXPECT_DOUBLE_EQ(next.linear.x, 0.0);
}

TEST_F(ControllerNodeTest, NewPathClearsGoalReached) {
  ControllerNode node;
  node.configure();
  node.handle_path(straight_path(5.0));
  node.handle_odom(odom_at(5.0, 0.0));
  ASSERT_TRUE(node.goal_reached());

  node.handle_path(straight_path(10.0));
  EXPECT_FALSE(node.goal_reached());
  const auto command = node.handle_odom(odom_at(5.0, 0.0));
  EXPECT_DOUBLE_EQ(command.linear.x, 0.5);
}

}  // namespace
