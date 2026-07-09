#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

#include "ars_core/geometry/pose2.hpp"
#include "ars_core/geometry/twist2.hpp"
#include "ars_core/planning/occupancy_grid.hpp"

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

/// Converts a ROS occupancy grid into the core planning grid.
///
/// Assumes an axis-aligned map (origin yaw = 0). Unknown cells (-1) and cells
/// at or above `occupied_threshold` are treated as occupied: the planner never
/// routes through space it cannot vouch for.
[[nodiscard]] inline core::planning::OccupancyGrid<double> to_grid(
    const nav_msgs::msg::OccupancyGrid& msg, std::int8_t occupied_threshold = 50) {
  core::planning::OccupancyGrid<double> grid{
      msg.info.width,
      msg.info.height,
      static_cast<double>(msg.info.resolution),
      {msg.info.origin.position.x, msg.info.origin.position.y}};
  for (std::size_t y = 0; y < msg.info.height; ++y) {
    for (std::size_t x = 0; x < msg.info.width; ++x) {
      const std::int8_t value = msg.data[y * msg.info.width + x];
      if (value < 0 || value >= occupied_threshold) {
        grid.set({static_cast<std::int64_t>(x), static_cast<std::int64_t>(y)},
                 core::planning::OccupancyGrid<double>::Cell::kOccupied);
      }
    }
  }
  return grid;
}

/// Builds a nav_msgs/Path from planar waypoints; every pose carries `header`
/// and an identity orientation.
[[nodiscard]] inline nav_msgs::msg::Path to_path_msg(
    const std::vector<core::geometry::Vec2<double>>& points, const std_msgs::msg::Header& header) {
  nav_msgs::msg::Path msg;
  msg.header = header;
  msg.poses.reserve(points.size());
  for (const auto& point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.orientation.w = 1.0;
    msg.poses.push_back(pose);
  }
  return msg;
}

/// Extracts the planar waypoints from a nav_msgs/Path.
[[nodiscard]] inline std::vector<core::geometry::Vec2<double>> from_msg(
    const nav_msgs::msg::Path& msg) {
  std::vector<core::geometry::Vec2<double>> points;
  points.reserve(msg.poses.size());
  for (const auto& pose : msg.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  return points;
}

}  // namespace ars::ros2::conversions
