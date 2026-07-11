#include <gtest/gtest.h>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "ars_ros2/mission_node.hpp"

namespace {

using ars::core::mission::Status;
using ars::ros2::MissionNode;

class MissionNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  static rclcpp::NodeOptions with_waypoints(std::vector<double> flat) {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter{"waypoints", std::move(flat)}});
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

TEST_F(MissionNodeTest, ConfiguresWithValidWaypoints) {
  MissionNode node{with_waypoints({1.0, 0.0, 2.0, 1.0})};
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(MissionNodeTest, RejectsOddWaypointList) {
  MissionNode node{with_waypoints({1.0, 0.0, 2.0})};
  EXPECT_EQ(node.configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(MissionNodeTest, DoesNotStartWithoutOdometry) {
  MissionNode node{with_waypoints({1.0, 0.0})};
  node.configure();
  node.activate();
  node.tick();
  EXPECT_EQ(node.mission_status(), Status::kRunning);
  EXPECT_EQ(node.current_waypoint(), 0U);
}

TEST_F(MissionNodeTest, CompletesMissionAsWaypointsAreReached) {
  MissionNode node{with_waypoints({1.0, 0.0, 2.0, 0.0})};
  node.configure();
  node.activate();

  node.handle_odom(odom_at(0.0, 0.0));
  node.tick();  // dispatches first goal, still en route
  EXPECT_EQ(node.mission_status(), Status::kRunning);
  EXPECT_EQ(node.current_waypoint(), 0U);

  node.handle_odom(odom_at(1.0, 0.0));  // first waypoint reached
  node.tick();                          // advances and dispatches the second goal
  EXPECT_EQ(node.mission_status(), Status::kRunning);
  EXPECT_EQ(node.current_waypoint(), 1U);

  node.handle_odom(odom_at(2.0, 0.0));  // second waypoint reached
  node.tick();
  EXPECT_EQ(node.mission_status(), Status::kSuccess);
  EXPECT_EQ(node.current_waypoint(), 2U);
}

TEST_F(MissionNodeTest, EmptyMissionSucceedsImmediately) {
  MissionNode node{with_waypoints({})};
  node.configure();
  node.activate();
  node.handle_odom(odom_at(0.0, 0.0));
  node.tick();
  EXPECT_EQ(node.mission_status(), Status::kSuccess);
}

TEST_F(MissionNodeTest, GoalToleranceIsRespected) {
  MissionNode node{with_waypoints({1.0, 0.0})};
  node.configure();
  node.activate();

  node.handle_odom(odom_at(0.0, 0.0));
  node.tick();
  node.handle_odom(odom_at(0.9, 0.1));  // within the default 0.15 m tolerance
  node.tick();
  EXPECT_EQ(node.mission_status(), Status::kSuccess);
}

TEST_F(MissionNodeTest, AbortsWhenNoProgressWithinTimeout) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter{"waypoints", std::vector<double>{5.0, 0.0}},
                               rclcpp::Parameter{"progress_timeout", 0.0000001},
                               rclcpp::Parameter{"max_attempts", 2}});
  MissionNode node{options};
  node.configure();
  node.activate();

  node.handle_odom(odom_at(0.0, 0.0));
  // The robot never moves: every tick expires the stall deadline until the
  // attempt budget is exhausted.
  for (int i = 0; i < 10 && node.mission_status() == Status::kRunning; ++i) {
    node.tick();
  }
  EXPECT_EQ(node.mission_status(), Status::kFailure);
  EXPECT_EQ(node.current_waypoint(), 0U);
}

TEST_F(MissionNodeTest, GoalSendFailsWhenNotActivated) {
  MissionNode node{with_waypoints({1.0, 0.0})};
  node.configure();
  // Never activated: the goal publisher refuses to publish, the send is
  // rejected, and the retry budget eventually aborts the mission.
  node.handle_odom(odom_at(0.0, 0.0));
  for (int i = 0; i < 10 && node.mission_status() == Status::kRunning; ++i) {
    node.tick();
  }
  EXPECT_EQ(node.mission_status(), Status::kFailure);
}

}  // namespace
