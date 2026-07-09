#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <string>

#include "ars_core/geometry/vec2.hpp"
#include "ars_core/planning/occupancy_grid.hpp"

namespace ars::ros2 {

/// Managed lifecycle node planning global paths on an occupancy grid.
///
/// On each goal it runs ars_core grid A* from the latest odometry position,
/// smooths the result, and publishes a nav_msgs/Path. The workspace comes from
/// parameters (a free rectangular grid) until a map arrives on the `map`
/// topic, which then takes precedence. All planning math lives in ars_core.
///
/// Parameters:
///   map_frame       frame of published paths           (default "odom")
///   map_width       default grid width  [cells]        (default 100)
///   map_height      default grid height [cells]        (default 100)
///   map_resolution  default grid resolution [m/cell]   (default 0.1)
///   map_origin_x    default grid origin x [m]          (default -5.0)
///   map_origin_y    default grid origin y [m]          (default -5.0)
class PlannerNode : public rclcpp_lifecycle::LifecycleNode {
 public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit PlannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  /// Message handlers; public so tests can drive the node without an executor.
  void handle_map(const nav_msgs::msg::OccupancyGrid& msg);
  void handle_odom(const nav_msgs::msg::Odometry& msg);
  void handle_goal(const geometry_msgs::msg::PoseStamped& msg);

  /// Plans a smoothed path between two world positions on the current grid.
  /// Returns nullopt when either endpoint is outside/blocked or no path
  /// exists. Exposed for testing and composition.
  [[nodiscard]] std::optional<nav_msgs::msg::Path> plan(core::geometry::Vec2<double> start,
                                                        core::geometry::Vec2<double> goal) const;

 private:
  std::optional<core::planning::OccupancyGrid<double>> grid_;
  std::optional<core::geometry::Vec2<double>> position_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> path_pub_;
  std::string map_frame_;
};

}  // namespace ars::ros2
