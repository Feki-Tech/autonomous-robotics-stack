#include "ars_ros2/mission_node.hpp"

#include <vector>

namespace ars::ros2 {

namespace mission = core::mission;

MissionNode::MissionNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("ars_mission", options) {
  declare_parameter("waypoints", std::vector<double>{});
  declare_parameter("goal_tolerance", 0.15);
  declare_parameter("progress_timeout", 30.0);
  declare_parameter("tick_period", 0.5);
  declare_parameter("max_attempts", 3);
}

MissionNode::CallbackReturn MissionNode::on_configure(const rclcpp_lifecycle::State&) {
  const auto flat = get_parameter("waypoints").as_double_array();
  if (flat.size() % 2 != 0) {
    RCLCPP_ERROR(get_logger(), "waypoints must contain an even number of values (x, y pairs)");
    return CallbackReturn::FAILURE;
  }
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  progress_timeout_ = get_parameter("progress_timeout").as_double();
  const auto max_attempts = get_parameter("max_attempts").as_int();
  if (goal_tolerance_ <= 0.0 || progress_timeout_ <= 0.0 || max_attempts <= 0) {
    RCLCPP_ERROR(get_logger(),
                 "goal_tolerance, progress_timeout, and max_attempts must be positive");
    return CallbackReturn::FAILURE;
  }

  std::vector<core::geometry::Vec2<double>> waypoints;
  waypoints.reserve(flat.size() / 2);
  for (std::size_t i = 0; i + 1 < flat.size(); i += 2) {
    waypoints.push_back({flat[i], flat[i + 1]});
  }
  mission_.emplace(std::move(waypoints), make_callbacks(),
                   mission::WaypointMissionConfig{.max_attempts_per_waypoint =
                                                      static_cast<std::size_t>(max_attempts)});
  status_ = mission::Status::kRunning;

  goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", rclcpp::QoS{1});
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS{},
      [this](const nav_msgs::msg::Odometry& msg) { handle_odom(msg); });
  RCLCPP_INFO(get_logger(), "configured with %zu waypoints", mission_->waypoint_count());
  return CallbackReturn::SUCCESS;
}

MissionNode::CallbackReturn MissionNode::on_activate(const rclcpp_lifecycle::State& state) {
  LifecycleNode::on_activate(state);
  const auto period = std::chrono::duration<double>{get_parameter("tick_period").as_double()};
  tick_timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                  [this] { tick(); });
  return CallbackReturn::SUCCESS;
}

MissionNode::CallbackReturn MissionNode::on_deactivate(const rclcpp_lifecycle::State& state) {
  tick_timer_.reset();
  if (mission_.has_value()) {
    mission_->halt();
  }
  LifecycleNode::on_deactivate(state);
  return CallbackReturn::SUCCESS;
}

MissionNode::CallbackReturn MissionNode::on_cleanup(const rclcpp_lifecycle::State&) {
  tick_timer_.reset();
  odom_sub_.reset();
  goal_pub_.reset();
  mission_.reset();
  position_.reset();
  active_goal_.reset();
  progress_deadline_.reset();
  return CallbackReturn::SUCCESS;
}

void MissionNode::handle_odom(const nav_msgs::msg::Odometry& msg) {
  position_ = core::geometry::Vec2<double>{msg.pose.pose.position.x, msg.pose.pose.position.y};
}

void MissionNode::tick() {
  if (!mission_.has_value() || status_ != mission::Status::kRunning) {
    return;
  }
  if (!position_.has_value()) {
    return;  // wait for odometry before starting the mission
  }
  status_ = mission_->tick();
  if (status_ == mission::Status::kSuccess) {
    RCLCPP_INFO(get_logger(), "mission complete (%zu waypoints)", mission_->waypoint_count());
  } else if (status_ == mission::Status::kFailure) {
    RCLCPP_ERROR(get_logger(), "mission aborted at waypoint %zu", mission_->current_waypoint());
  }
}

core::mission::WaypointMissionCallbacks MissionNode::make_callbacks() {
  return {
      .send_goal =
          [this](core::geometry::Vec2<double> goal) {
            if (goal_pub_ == nullptr || !goal_pub_->is_activated()) {
              return false;
            }
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = now();
            msg.header.frame_id = "odom";
            msg.pose.position.x = goal.x;
            msg.pose.position.y = goal.y;
            msg.pose.orientation.w = 1.0;
            goal_pub_->publish(msg);

            active_goal_ = goal;
            best_distance_ = position_.has_value() ? (goal - *position_).norm() : 0.0;
            progress_deadline_ = now() + rclcpp::Duration::from_seconds(progress_timeout_);
            RCLCPP_INFO(get_logger(), "waypoint %zu: goal (%.2f, %.2f)",
                        mission_->current_waypoint(), goal.x, goal.y);
            return true;
          },
      .goal_reached =
          [this] {
            if (!active_goal_.has_value() || !position_.has_value()) {
              return false;
            }
            const double distance = (*active_goal_ - *position_).norm();
            // Any measurable progress extends the stall deadline.
            if (distance < best_distance_ - 0.01) {
              best_distance_ = distance;
              progress_deadline_ = now() + rclcpp::Duration::from_seconds(progress_timeout_);
            }
            return distance <= goal_tolerance_;
          },
      .navigation_failed =
          [this] { return progress_deadline_.has_value() && now() >= *progress_deadline_; },
  };
}

}  // namespace ars::ros2
