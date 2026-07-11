#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <string>

#include "ars_core/mission/waypoint_mission.hpp"

namespace ars::ros2 {

/// Managed lifecycle node executing waypoint missions with a behavior tree.
///
/// On activation it builds an ars::core::mission::WaypointMission from the
/// `waypoints` parameter and ticks it on a timer. Goals are dispatched to the
/// planner via `goal_pose`; arrival is detected from odometry, and a leg is
/// declared failed when no progress is made within `progress_timeout`. Failed
/// legs are retried by the tree's recovery policy before the mission aborts.
///
/// Parameters:
///   waypoints            flat [x0, y0, x1, y1, ...] list [m]
///   goal_tolerance       arrival distance [m]              (default 0.15)
///   progress_timeout     stall budget per leg [s]          (default 30.0)
///   tick_period          tree tick period [s]              (default 0.5)
///   max_attempts         attempts per waypoint             (default 3)
class MissionNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit MissionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /// Feeds one odometry sample; public so tests can drive the node without an
  /// executor.
  void handle_odom(const nav_msgs::msg::Odometry& msg);

  /// Ticks the mission tree once (also called by the activation timer).
  void tick();

  [[nodiscard]] core::mission::Status mission_status() const { return status_; }
  [[nodiscard]] std::size_t current_waypoint() const {
    return mission_.has_value() ? mission_->current_waypoint() : 0;
  }

 private:
  [[nodiscard]] core::mission::WaypointMissionCallbacks make_callbacks();

  std::optional<core::mission::WaypointMission> mission_;
  core::mission::Status status_{core::mission::Status::kRunning};
  std::optional<core::geometry::Vec2<double>> position_;
  std::optional<core::geometry::Vec2<double>> active_goal_;
  double goal_tolerance_{0.15};
  double progress_timeout_{30.0};
  double best_distance_{0.0};
  std::optional<rclcpp::Time> progress_deadline_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> goal_pub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
};

}  // namespace ars::ros2
