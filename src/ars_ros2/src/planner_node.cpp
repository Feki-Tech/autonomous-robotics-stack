#include "ars_ros2/planner_node.hpp"

#include <vector>

#include "ars_core/planning/astar.hpp"
#include "ars_core/planning/path_smoother.hpp"
#include "ars_ros2/conversions.hpp"

namespace ars::ros2 {

PlannerNode::PlannerNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("ars_planner", options) {
  declare_parameter("map_frame", "odom");
  declare_parameter("map_width", 100);
  declare_parameter("map_height", 100);
  declare_parameter("map_resolution", 0.1);
  declare_parameter("map_origin_x", -5.0);
  declare_parameter("map_origin_y", -5.0);
}

PlannerNode::CallbackReturn PlannerNode::on_configure(const rclcpp_lifecycle::State&) {
  const auto width = get_parameter("map_width").as_int();
  const auto height = get_parameter("map_height").as_int();
  const auto resolution = get_parameter("map_resolution").as_double();
  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    RCLCPP_ERROR(get_logger(), "map_width, map_height, and map_resolution must be positive");
    return CallbackReturn::FAILURE;
  }
  map_frame_ = get_parameter("map_frame").as_string();
  grid_.emplace(static_cast<std::size_t>(width), static_cast<std::size_t>(height), resolution,
                core::geometry::Vec2<double>{get_parameter("map_origin_x").as_double(),
                                             get_parameter("map_origin_y").as_double()});

  path_pub_ = create_publisher<nav_msgs::msg::Path>("path", rclcpp::QoS{1}.transient_local());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS{1}.transient_local(),
      [this](const nav_msgs::msg::OccupancyGrid& msg) { handle_map(msg); });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS{},
      [this](const nav_msgs::msg::Odometry& msg) { handle_odom(msg); });
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS{1},
      [this](const geometry_msgs::msg::PoseStamped& msg) { handle_goal(msg); });
  RCLCPP_INFO(get_logger(), "configured (%ldx%ld cells @ %.2f m)", width, height, resolution);
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn PlannerNode::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn PlannerNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

PlannerNode::CallbackReturn PlannerNode::on_cleanup(const rclcpp_lifecycle::State&) {
  map_sub_.reset();
  odom_sub_.reset();
  goal_sub_.reset();
  path_pub_.reset();
  grid_.reset();
  position_.reset();
  return CallbackReturn::SUCCESS;
}

void PlannerNode::handle_map(const nav_msgs::msg::OccupancyGrid& msg) {
  grid_ = conversions::to_grid(msg);
  RCLCPP_INFO(get_logger(), "map received (%ux%u cells @ %.2f m)", msg.info.width, msg.info.height,
              msg.info.resolution);
}

void PlannerNode::handle_odom(const nav_msgs::msg::Odometry& msg) {
  position_ = core::geometry::Vec2<double>{msg.pose.pose.position.x, msg.pose.pose.position.y};
}

void PlannerNode::handle_goal(const geometry_msgs::msg::PoseStamped& msg) {
  if (!position_.has_value()) {
    RCLCPP_WARN(get_logger(), "goal rejected: no odometry received yet");
    return;
  }
  const auto path = plan(*position_, {msg.pose.position.x, msg.pose.position.y});
  if (!path.has_value()) {
    RCLCPP_WARN(get_logger(), "no path from (%.2f, %.2f) to (%.2f, %.2f)", position_->x,
                position_->y, msg.pose.position.x, msg.pose.position.y);
    return;
  }
  if (path_pub_ != nullptr && path_pub_->is_activated()) {
    path_pub_->publish(*path);
  }
  RCLCPP_INFO(get_logger(), "planned path with %zu poses", path->poses.size());
}

std::optional<nav_msgs::msg::Path> PlannerNode::plan(core::geometry::Vec2<double> start,
                                                     core::geometry::Vec2<double> goal) const {
  if (!grid_.has_value()) {
    return std::nullopt;
  }
  const auto start_cell = grid_->world_to_cell(start);
  const auto goal_cell = grid_->world_to_cell(goal);
  if (!start_cell.has_value() || !goal_cell.has_value()) {
    return std::nullopt;
  }
  const auto cells = core::planning::find_path(*grid_, *start_cell, *goal_cell);
  if (!cells.has_value()) {
    return std::nullopt;
  }

  std::vector<core::geometry::Vec2<double>> points;
  points.reserve(cells->size());
  for (const auto cell : *cells) {
    points.push_back(grid_->cell_to_world(cell));
  }
  // Anchor the endpoints to the exact requested positions, not cell centers.
  points.front() = start;
  points.back() = goal;

  std_msgs::msg::Header header;
  header.stamp = now();
  header.frame_id = map_frame_;
  return conversions::to_path_msg(core::planning::smooth_path(points), header);
}

}  // namespace ars::ros2
