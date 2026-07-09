#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>
#include <vector>

#include "ars_core/geometry/pose2.hpp"
#include "ars_core/geometry/twist2.hpp"
#include "ars_core/geometry/vec2.hpp"

namespace ars::core::control {

/// Configuration for the pure pursuit path tracker.
template <std::floating_point T = double>
struct PurePursuitConfig {
  T cruise_speed{0.5};      ///< nominal forward speed [m/s]
  T lookahead_gain{1.0};    ///< lookahead = gain * speed, adaptive with velocity
  T min_lookahead{0.3};     ///< [m]
  T max_lookahead{2.0};     ///< [m]
  T goal_tolerance{0.05};   ///< distance considered "arrived" [m]
  T max_angular_rate{2.0};  ///< command clamp [rad/s]
};

/// Command produced by one tracking step.
template <std::floating_point T = double>
struct PurePursuitCommand {
  geometry::Twist2<T> twist{};
  bool goal_reached{false};
};

/// Pure pursuit path tracking for differential-drive robots.
///
/// Chases a lookahead point that slides along the path; the commanded
/// curvature is the circle through the robot and that point:
/// kappa = 2 * y_lookahead / L^2 (in the body frame). The lookahead distance
/// scales with speed for stability at pace and agility when slow.
template <std::floating_point T = double>
class PurePursuit {
 public:
  PurePursuit(std::vector<geometry::Vec2<T>> path, PurePursuitConfig<T> config = {})
      : path_{std::move(path)}, config_{config} {}

  [[nodiscard]] const std::vector<geometry::Vec2<T>>& path() const { return path_; }

  /// Computes the velocity command for the current pose.
  [[nodiscard]] PurePursuitCommand<T> track(const geometry::Pose2<T>& pose) const {
    if (path_.empty()) {
      return {.goal_reached = true};
    }
    const auto& goal = path_.back();
    const auto to_goal = goal - pose.translation();
    if (to_goal.norm() <= config_.goal_tolerance) {
      return {.goal_reached = true};
    }

    const T lookahead = std::clamp(config_.lookahead_gain * config_.cruise_speed,
                                   config_.min_lookahead, config_.max_lookahead);
    const auto target = find_lookahead_point(pose.translation(), lookahead);

    // Target in the body frame; curvature of the arc through it.
    const auto local = pose.inverse().transform(target);
    const T distance_sq = local.norm_squared();
    T curvature{};
    if (distance_sq > T{0}) {
      curvature = T{2} * local.y / distance_sq;
    }

    const T angular = std::clamp(config_.cruise_speed * curvature, -config_.max_angular_rate,
                                 config_.max_angular_rate);
    return {.twist = {config_.cruise_speed, T{0}, angular}};
  }

 private:
  /// Point on the path at `lookahead` distance from the robot: the farthest
  /// path-segment intersection with the lookahead circle, or the closest path
  /// point if the path is entirely outside, or the goal if inside the circle.
  [[nodiscard]] geometry::Vec2<T> find_lookahead_point(const geometry::Vec2<T>& position,
                                                       T lookahead) const {
    std::optional<geometry::Vec2<T>> best;
    for (std::size_t i = path_.size() - 1; i > 0; --i) {
      const auto& a = path_[i - 1];
      const auto& b = path_[i];
      if (const auto hit = circle_segment_intersection(position, lookahead, a, b)) {
        best = hit;
        break;  // Latest segment first: never track backwards.
      }
    }
    if (best.has_value()) {
      return *best;
    }
    // No intersection: head for the closest path vertex (recovers from
    // large deviations), or the goal when already within the circle.
    const auto& goal = path_.back();
    if ((goal - position).norm() < lookahead) {
      return goal;
    }
    return *std::ranges::min_element(path_, {}, [&position](const geometry::Vec2<T>& p) {
      return (p - position).norm_squared();
    });
  }

  /// Farthest intersection (toward b) of segment [a,b] with the circle.
  [[nodiscard]] static std::optional<geometry::Vec2<T>> circle_segment_intersection(
      const geometry::Vec2<T>& center, T radius, const geometry::Vec2<T>& a,
      const geometry::Vec2<T>& b) {
    const auto d = b - a;
    const auto f = a - center;
    const T qa = d.norm_squared();
    if (qa == T{0}) {
      return std::nullopt;
    }
    const T qb = T{2} * f.dot(d);
    const T qc = f.norm_squared() - radius * radius;
    const T discriminant = qb * qb - T{4} * qa * qc;
    if (discriminant < T{0}) {
      return std::nullopt;
    }
    const T sqrt_disc = std::sqrt(discriminant);
    // Prefer the exit point (larger t) to keep progressing along the path.
    for (const T t : {(-qb + sqrt_disc) / (T{2} * qa), (-qb - sqrt_disc) / (T{2} * qa)}) {
      if (t >= T{0} && t <= T{1}) {
        return a + d * t;
      }
    }
    return std::nullopt;
  }

  std::vector<geometry::Vec2<T>> path_;
  PurePursuitConfig<T> config_;
};

}  // namespace ars::core::control
