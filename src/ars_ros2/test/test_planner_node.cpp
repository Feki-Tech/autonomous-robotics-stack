#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <utility>
#include <vector>

#include "ars_ros2/planner_node.hpp"

namespace {

using ars::ros2::PlannerNode;

class PlannerNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::NodeOptions with_params(std::vector<rclcpp::Parameter> params) {
    rclcpp::NodeOptions options;
    options.parameter_overrides(std::move(params));
    return options;
  }

  static nav_msgs::msg::Odometry odom_at(double x, double y) {
    nav_msgs::msg::Odometry msg;
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.w = 1.0;
    return msg;
  }
};

TEST_F(PlannerNodeTest, ConfiguresWithDefaultParameters) {
  PlannerNode node;
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(PlannerNodeTest, RejectsNonPositiveResolution) {
  PlannerNode node{with_params({rclcpp::Parameter{"map_resolution", 0.0}})};
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(PlannerNodeTest, PlansAcrossFreeGrid) {
  PlannerNode node;
  node.configure();
  const auto path = node.plan({0.0, 0.0}, {3.0, 2.0});
  ASSERT_TRUE(path.has_value());
  ASSERT_GE(path->poses.size(), 2U);
  EXPECT_DOUBLE_EQ(path->poses.front().pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(path->poses.front().pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(path->poses.back().pose.position.x, 3.0);
  EXPECT_DOUBLE_EQ(path->poses.back().pose.position.y, 2.0);
  EXPECT_EQ(path->header.frame_id, "odom");
}

TEST_F(PlannerNodeTest, RejectsGoalOutsideGrid) {
  PlannerNode node;
  node.configure();
  EXPECT_FALSE(node.plan({0.0, 0.0}, {100.0, 100.0}).has_value());
}

TEST_F(PlannerNodeTest, MapTopicOverridesParameterGrid) {
  PlannerNode node;
  node.configure();

  // A 10x10 map at 1 m resolution with a full-height wall at x = 5,
  // except the origin differs from the parameter default.
  nav_msgs::msg::OccupancyGrid map;
  map.info.width = 10;
  map.info.height = 10;
  map.info.resolution = 1.0;
  map.info.origin.position.x = 0.0;
  map.info.origin.position.y = 0.0;
  map.data.assign(100, 0);
  for (std::size_t y = 0; y < 10; ++y) {
    map.data[y * 10 + 5] = 100;
  }
  node.handle_map(map);

  EXPECT_FALSE(node.plan({1.5, 1.5}, {8.5, 1.5}).has_value());
  EXPECT_TRUE(node.plan({1.5, 1.5}, {4.0, 8.0}).has_value());
}

TEST_F(PlannerNodeTest, GoalWithoutOdometryDoesNotCrash) {
  PlannerNode node;
  node.configure();
  node.activate();
  geometry_msgs::msg::PoseStamped goal;
  goal.pose.position.x = 1.0;
  node.handle_goal(goal);  // no odometry yet: must be rejected gracefully
  node.handle_odom(odom_at(0.0, 0.0));
  node.handle_goal(goal);
  SUCCEED();
}

}  // namespace
