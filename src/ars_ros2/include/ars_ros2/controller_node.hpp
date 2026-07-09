#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "ars_core/control/pure_pursuit.hpp"

namespace ars::ros2 {

/// Managed lifecycle node tracking paths with pure pursuit.
///
/// Latches the most recent nav_msgs/Path and, on every odometry sample,
/// publishes the geometry_msgs/Twist command produced by
/// ars::core::control::PurePursuit. Commands zero velocity when there is no
/// path or the goal has been reached. All control math lives in ars_core.
///
/// Parameters:
///   cruise_speed      nominal forward speed [m/s]        (default 0.5)
///   lookahead_gain    lookahead = gain * speed           (default 1.0)
///   min_lookahead     [m]                                (default 0.3)
///   max_lookahead     [m]                                (default 2.0)
///   goal_tolerance    arrival distance [m]               (default 0.05)
///   max_angular_rate  command clamp [rad/s]              (default 2.0)
class ControllerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit ControllerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /// Latches a new path to track; public so tests can drive the node without
  /// an executor.
  void handle_path(const nav_msgs::msg::Path& msg);

  /// Computes (and, when active, publishes) the command for one odometry
  /// sample. The command is returned for testing and composition.
  geometry_msgs::msg::Twist handle_odom(const nav_msgs::msg::Odometry& msg);

  /// True once the latched path's goal has been reached.
  [[nodiscard]] bool goal_reached() const { return goal_reached_; }

 private:
  std::optional<core::control::PurePursuit<double>> tracker_;
  core::control::PurePursuitConfig<double> config_;
  bool goal_reached_{false};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> cmd_pub_;
};

}  // namespace ars::ros2
