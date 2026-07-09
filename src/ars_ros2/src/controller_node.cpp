#include "ars_ros2/controller_node.hpp"

#include "ars_ros2/conversions.hpp"

namespace ars::ros2 {

ControllerNode::ControllerNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("ars_controller", options) {
  declare_parameter("cruise_speed", 0.5);
  declare_parameter("lookahead_gain", 1.0);
  declare_parameter("min_lookahead", 0.3);
  declare_parameter("max_lookahead", 2.0);
  declare_parameter("goal_tolerance", 0.05);
  declare_parameter("max_angular_rate", 2.0);
}

ControllerNode::CallbackReturn ControllerNode::on_configure(const rclcpp_lifecycle::State&) {
  config_ = {.cruise_speed = get_parameter("cruise_speed").as_double(),
             .lookahead_gain = get_parameter("lookahead_gain").as_double(),
             .min_lookahead = get_parameter("min_lookahead").as_double(),
             .max_lookahead = get_parameter("max_lookahead").as_double(),
             .goal_tolerance = get_parameter("goal_tolerance").as_double(),
             .max_angular_rate = get_parameter("max_angular_rate").as_double()};
  if (config_.cruise_speed <= 0.0 || config_.min_lookahead <= 0.0 ||
      config_.max_lookahead < config_.min_lookahead || config_.goal_tolerance <= 0.0 ||
      config_.max_angular_rate <= 0.0) {
    RCLCPP_ERROR(get_logger(), "invalid pure pursuit configuration");
    return CallbackReturn::FAILURE;
  }

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS{1});
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::Path& msg) { handle_path(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS{},
      [this](const nav_msgs::msg::Odometry& msg) { handle_odom(msg); });
  RCLCPP_INFO(get_logger(), "configured (cruise=%.2f m/s)", config_.cruise_speed);
  return CallbackReturn::SUCCESS;
}

ControllerNode::CallbackReturn ControllerNode::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);
  return CallbackReturn::SUCCESS;
}

ControllerNode::CallbackReturn ControllerNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

ControllerNode::CallbackReturn ControllerNode::on_cleanup(const rclcpp_lifecycle::State&) {
  path_sub_.reset();
  odom_sub_.reset();
  cmd_pub_.reset();
  tracker_.reset();
  goal_reached_ = false;
  return CallbackReturn::SUCCESS;
}

void ControllerNode::handle_path(const nav_msgs::msg::Path& msg) {
  tracker_.emplace(conversions::from_msg(msg), config_);
  goal_reached_ = false;
  RCLCPP_INFO(get_logger(), "tracking path with %zu poses", msg.poses.size());
}

geometry_msgs::msg::Twist ControllerNode::handle_odom(const nav_msgs::msg::Odometry& msg) {
  geometry_msgs::msg::Twist command;  // zero unless there is work to do
  if (tracker_.has_value() && !goal_reached_) {
    const auto result = tracker_->track(conversions::from_msg(msg.pose.pose));
    if (result.goal_reached) {
      goal_reached_ = true;
      RCLCPP_INFO(get_logger(), "goal reached");
    } else {
      command = conversions::to_msg(result.twist);
    }
  }
  if (cmd_pub_ != nullptr && cmd_pub_->is_activated()) {
    cmd_pub_->publish(command);
  }
  return command;
}

}  // namespace ars::ros2
