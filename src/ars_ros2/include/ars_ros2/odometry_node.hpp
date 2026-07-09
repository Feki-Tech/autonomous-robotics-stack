#pragma once

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "ars_core/estimation/diff_drive_odometry.hpp"

namespace ars::ros2 {

/// Managed lifecycle node publishing dead-reckoning odometry.
///
/// Subscribes to wheel joint states, feeds the ROS-agnostic
/// ars::core::estimation::DiffDriveOdometry, and publishes nav_msgs/Odometry.
/// All robot math lives in ars_core; this class only moves messages.
///
/// Parameters:
///   wheel_radius      [m]   (default 0.1)
///   wheel_separation  [m]   (default 0.5)
///   left_joint        name of the left wheel joint  (default "left_wheel")
///   right_joint       name of the right wheel joint (default "right_wheel")
///   odom_frame        header frame of published odometry (default "odom")
///   base_frame        child frame of published odometry (default "base_link")
class OdometryNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit OdometryNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /// Latest pose estimate (exposed for testing and composition).
  [[nodiscard]] core::geometry::Pose2<double> pose() const;

  /// Processes one joint-state sample; public so tests can drive the node
  /// without an executor.
  void handle_joint_state(const sensor_msgs::msg::JointState& msg);

 private:
  std::optional<core::estimation::DiffDriveOdometry<double>> odometry_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> odom_pub_;
  std::string left_joint_;
  std::string right_joint_;
  std::string odom_frame_;
  std::string base_frame_;
  std::optional<rclcpp::Time> last_stamp_;
};

}  // namespace ars::ros2
