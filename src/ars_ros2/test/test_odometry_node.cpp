#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ars_ros2/odometry_node.hpp"

namespace ars::ros2 {
namespace {

sensor_msgs::msg::JointState make_joint_state(double left, double right, double t_seconds) {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = rclcpp::Time{static_cast<std::int64_t>(t_seconds * 1e9)};
  msg.name = {"left_wheel", "right_wheel"};
  msg.velocity = {left, right};
  return msg;
}

class OdometryNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

TEST_F(OdometryNodeTest, LifecycleTransitionsSucceed) {
  OdometryNode node;
  EXPECT_EQ(node.get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  node.configure();
  EXPECT_EQ(node.get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  node.activate();
  EXPECT_EQ(node.get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  node.deactivate();
  node.cleanup();
  EXPECT_EQ(node.get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(OdometryNodeTest, RejectsInvalidParameters) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"wheel_radius", -1.0}});
  OdometryNode node{options};
  node.configure();
  EXPECT_EQ(node.get_current_state().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(OdometryNodeTest, IntegratesJointStatesIntoPose) {
  OdometryNode node;
  node.configure();
  node.activate();
  // 10 rad/s on both wheels with r=0.1 -> 1 m/s straight; 1 s total.
  for (int i = 0; i <= 100; ++i) {
    node.handle_joint_state(make_joint_state(10.0, 10.0, 0.01 * i));
  }
  EXPECT_NEAR(node.pose().x(), 1.0, 1e-9);
  EXPECT_NEAR(node.pose().y(), 0.0, 1e-9);
}

TEST_F(OdometryNodeTest, IgnoresUnknownJointsAndBadTimestamps) {
  OdometryNode node;
  node.configure();
  node.activate();

  sensor_msgs::msg::JointState wrong_names = make_joint_state(10.0, 10.0, 0.0);
  wrong_names.name = {"a", "b"};
  node.handle_joint_state(wrong_names);

  // First valid sample sets the clock; a repeated stamp (dt=0) must not move the pose.
  node.handle_joint_state(make_joint_state(10.0, 10.0, 1.0));
  node.handle_joint_state(make_joint_state(10.0, 10.0, 1.0));
  EXPECT_DOUBLE_EQ(node.pose().x(), 0.0);
}

}  // namespace
}  // namespace ars::ros2
