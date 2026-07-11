#pragma once

#include <cstddef>
#include <functional>
#include <utility>
#include <vector>

#include "ars_core/geometry/vec2.hpp"
#include "ars_core/mission/behavior_tree.hpp"

namespace ars::core::mission {

/// Callbacks through which a waypoint mission drives the rest of the stack.
/// The mission layer never talks to ROS directly (ADR-0002): the adapter
/// wires these to goal publication and odometry/controller feedback.
struct WaypointMissionCallbacks {
  /// Requests navigation toward a waypoint (e.g. publishes a goal).
  /// Returns false when the request itself fails (e.g. no path found).
  std::function<bool(geometry::Vec2<double>)> send_goal;
  /// True once the active goal has been reached.
  std::function<bool()> goal_reached;
  /// True when tracking has failed and a recovery/retry is warranted.
  std::function<bool()> navigation_failed;
};

/// Configuration for a waypoint mission.
struct WaypointMissionConfig {
  std::size_t max_attempts_per_waypoint{3};
};

/// A waypoint-following mission as a behavior tree.
///
/// For each waypoint: send the goal, then tick until the goal is reached
/// (success) or navigation fails (failure). Failures are retried up to
/// `max_attempts_per_waypoint` times before the mission aborts. Tick the
/// mission periodically; it reports kRunning until terminal.
class WaypointMission {
 public:
  WaypointMission(std::vector<geometry::Vec2<double>> waypoints, WaypointMissionCallbacks callbacks,
                  WaypointMissionConfig config = {})
      : waypoints_{std::move(waypoints)}, callbacks_{std::move(callbacks)}, root_{build(config)} {}

  /// Advances the mission one tick.
  Status tick() { return root_->tick(); }

  /// Preempts the mission; the next tick restarts from the first waypoint.
  void halt() {
    root_->halt();
    current_waypoint_ = 0;
  }

  [[nodiscard]] std::size_t waypoint_count() const { return waypoints_.size(); }

  /// Index of the waypoint currently being pursued (== waypoint_count() when
  /// the mission has completed).
  [[nodiscard]] std::size_t current_waypoint() const { return current_waypoint_; }

 private:
  [[nodiscard]] NodePtr build(const WaypointMissionConfig& config) {
    std::vector<NodePtr> legs;
    legs.reserve(waypoints_.size());
    for (std::size_t i = 0; i < waypoints_.size(); ++i) {
      legs.push_back(retry("retry_waypoint",
                           sequence("navigate_waypoint",
                                    action("send_goal",
                                           [this, i] {
                                             current_waypoint_ = i;
                                             return callbacks_.send_goal(waypoints_[i])
                                                        ? Status::kSuccess
                                                        : Status::kFailure;
                                           }),
                                    action("await_arrival",
                                           [this] {
                                             if (callbacks_.goal_reached()) {
                                               return Status::kSuccess;
                                             }
                                             if (callbacks_.navigation_failed()) {
                                               return Status::kFailure;
                                             }
                                             return Status::kRunning;
                                           })),
                           config.max_attempts_per_waypoint));
    }
    auto root = std::make_unique<Sequence>("mission", std::move(legs));
    return sequence("mission_with_epilogue", std::move(root), action("complete", [this] {
                      current_waypoint_ = waypoints_.size();
                      return Status::kSuccess;
                    }));
  }

  std::vector<geometry::Vec2<double>> waypoints_;
  WaypointMissionCallbacks callbacks_;
  std::size_t current_waypoint_{0};
  NodePtr root_;
};

}  // namespace ars::core::mission
