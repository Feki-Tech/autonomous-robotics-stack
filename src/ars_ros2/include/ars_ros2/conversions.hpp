#pragma once

#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "ars_core/geometry/pose2.hpp"
#include "ars_core/geometry/twist2.hpp"

/// Message conversions at the adapter boundary (ADR-0002).
///
/// The planar core state maps into full 3D ROS messages with z, roll, and
/// pitch fixed at zero; the inverse conversions project back by extracting
/// yaw and dropping the out-of-plane components.
namespace ars::ros2::conversions {

[[nodiscard]] inline geometry_msgs::msg::Pose to_msg(const core::geometry::Pose2<double>& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.x();
  msg.position.y = pose.y();
  const double half_yaw = pose.rotation().radians() / 2.0;
  msg.orientation.z = std::sin(half_yaw);
  msg.orientation.w = std::cos(half_yaw);
  return msg;
}

[[nodiscard]] inline core::geometry::Pose2<double> from_msg(const geometry_msgs::msg::Pose& msg) {
  // Yaw from quaternion; valid for arbitrary (normalized) orientations.
  const auto& q = msg.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  return {msg.position.x, msg.position.y, yaw};
}

[[nodiscard]] inline geometry_msgs::msg::Twist to_msg(const core::geometry::Twist2<double>& twist) {
  geometry_msgs::msg::Twist msg;
  msg.linear.x = twist.linear.x;
  msg.linear.y = twist.linear.y;
  msg.angular.z = twist.angular;
  return msg;
}

[[nodiscard]] inline core::geometry::Twist2<double> from_msg(const geometry_msgs::msg::Twist& msg) {
  return {msg.linear.x, msg.linear.y, msg.angular.z};
}

}  // namespace ars::ros2::conversions
