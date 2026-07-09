#include "ars_ros2/odometry_node.hpp"

#include <algorithm>

#include "ars_ros2/conversions.hpp"

namespace ars::ros2 {

OdometryNode::OdometryNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("ars_odometry", options) {
  declare_parameter("wheel_radius", 0.1);
  declare_parameter("wheel_separation", 0.5);
  declare_parameter("left_joint", "left_wheel");
  declare_parameter("right_joint", "right_wheel");
  declare_parameter("odom_frame", "odom");
  declare_parameter("base_frame", "base_link");
}

OdometryNode::CallbackReturn OdometryNode::on_configure(const rclcpp_lifecycle::State&) {
  const core::estimation::DiffDriveModel<double> model{
      get_parameter("wheel_radius").as_double(), get_parameter("wheel_separation").as_double()};
  if (model.wheel_radius <= 0.0 || model.wheel_separation <= 0.0) {
    RCLCPP_ERROR(get_logger(), "wheel_radius and wheel_separation must be positive");
    return CallbackReturn::FAILURE;
  }
  odometry_.emplace(model);
  left_joint_ = get_parameter("left_joint").as_string();
  right_joint_ = get_parameter("right_joint").as_string();
  odom_frame_ = get_parameter("odom_frame").as_string();
  base_frame_ = get_parameter("base_frame").as_string();

  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS{});
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", rclcpp::SensorDataQoS{},
      [this](const sensor_msgs::msg::JointState& msg) { handle_joint_state(msg); });
  RCLCPP_INFO(get_logger(), "configured (r=%.3f m, b=%.3f m)", model.wheel_radius,
              model.wheel_separation);
  return CallbackReturn::SUCCESS;
}

OdometryNode::CallbackReturn OdometryNode::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);
  return CallbackReturn::SUCCESS;
}

OdometryNode::CallbackReturn OdometryNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

OdometryNode::CallbackReturn OdometryNode::on_cleanup(const rclcpp_lifecycle::State&) {
  joint_state_sub_.reset();
  odom_pub_.reset();
  odometry_.reset();
  last_stamp_.reset();
  return CallbackReturn::SUCCESS;
}

core::geometry::Pose2<double> OdometryNode::pose() const {
  return odometry_.has_value() ? odometry_->pose() : core::geometry::Pose2<double>{};
}

void OdometryNode::handle_joint_state(const sensor_msgs::msg::JointState& msg) {
  if (!odometry_.has_value()) {
    return;
  }
  const auto left_it = std::ranges::find(msg.name, left_joint_);
  const auto right_it = std::ranges::find(msg.name, right_joint_);
  if (left_it == msg.name.end() || right_it == msg.name.end()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "joint states missing '%s'/'%s'",
                         left_joint_.c_str(), right_joint_.c_str());
    return;
  }
  const auto left_index = static_cast<std::size_t>(left_it - msg.name.begin());
  const auto right_index = static_cast<std::size_t>(right_it - msg.name.begin());
  if (msg.velocity.size() <= std::max(left_index, right_index)) {
    return;
  }

  const rclcpp::Time stamp{msg.header.stamp};
  if (last_stamp_.has_value()) {
    const double dt = (stamp - *last_stamp_).seconds();
    if (dt > 0.0) {
      odometry_->update(msg.velocity[left_index], msg.velocity[right_index], dt);
    }
  }
  last_stamp_ = stamp;

  if (odom_pub_ == nullptr || !odom_pub_->is_activated()) {
    return;
  }
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = conversions::to_msg(odometry_->pose());
  odom.twist.twist = conversions::to_msg(odometry_->velocity());
  odom_pub_->publish(odom);
}

}  // namespace ars::ros2
