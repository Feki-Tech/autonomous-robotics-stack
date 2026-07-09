#include <gtest/gtest.h>

#include <cmath>
#include <numbers>
#include <vector>

#include "ars_core/control/pure_pursuit.hpp"
#include "ars_core/estimation/diff_drive_odometry.hpp"

namespace ars::core::control {
namespace {

constexpr double kPi = std::numbers::pi;

using geometry::Pose2;
using geometry::Vec2;
using Path = std::vector<Vec2<double>>;

Path straight_path(double length, double step) {
  Path path;
  for (double x = 0.0; x <= length; x += step) {
    path.push_back({x, 0.0});
  }
  return path;
}

TEST(PurePursuitTest, EmptyPathIsGoalReached) {
  const PurePursuit<double> tracker{Path{}};
  EXPECT_TRUE(tracker.track(Pose2{}).goal_reached);
}

TEST(PurePursuitTest, GoalToleranceStops) {
  PurePursuitConfig<double> config;
  config.goal_tolerance = 0.1;
  const PurePursuit tracker{straight_path(5.0, 0.5), config};
  EXPECT_TRUE(tracker.track(Pose2{4.95, 0.0, 0.0}).goal_reached);
  EXPECT_FALSE(tracker.track(Pose2{4.0, 0.0, 0.0}).goal_reached);
}

TEST(PurePursuitTest, OnPathDrivesStraight) {
  const PurePursuit tracker{straight_path(10.0, 0.5)};
  const auto cmd = tracker.track(Pose2{2.0, 0.0, 0.0});
  EXPECT_FALSE(cmd.goal_reached);
  EXPECT_GT(cmd.twist.linear.x, 0.0);
  EXPECT_NEAR(cmd.twist.angular, 0.0, 1e-9);
}

TEST(PurePursuitTest, SteersTowardPathFromLeftOffset) {
  const PurePursuit tracker{straight_path(10.0, 0.5)};
  // Robot left of the path (positive y), heading along it: must steer right.
  const auto cmd = tracker.track(Pose2{2.0, 0.5, 0.0});
  EXPECT_LT(cmd.twist.angular, 0.0);
}

TEST(PurePursuitTest, SteersTowardPathFromRightOffset) {
  const PurePursuit tracker{straight_path(10.0, 0.5)};
  const auto cmd = tracker.track(Pose2{2.0, -0.5, 0.0});
  EXPECT_GT(cmd.twist.angular, 0.0);
}

TEST(PurePursuitTest, CurvatureMatchesAnalyticFormula) {
  PurePursuitConfig<double> config;
  config.cruise_speed = 1.0;
  config.lookahead_gain = 1.0;
  config.min_lookahead = 1.0;
  config.max_lookahead = 1.0;
  // Path far enough that the lookahead circle cuts the segment exactly once.
  const Path path{{0.0, 0.0}, {10.0, 0.0}};
  const PurePursuit tracker{path, config};
  // Robot at origin offset 0.5 above the path, heading along x.
  const Pose2 pose{0.0, 0.5, 0.0};
  // Lookahead point: circle radius 1 centered at (0,0.5) intersects y=0 at
  // x = sqrt(1 - 0.25) = sqrt(0.75). Body-frame target y = -0.5.
  const double expected_curvature = 2.0 * (-0.5) / 1.0;  // 2y/L^2, L=1
  const auto cmd = tracker.track(pose);
  EXPECT_NEAR(cmd.twist.angular, expected_curvature * config.cruise_speed, 1e-9);
}

TEST(PurePursuitTest, AngularRateIsClamped) {
  PurePursuitConfig<double> config;
  config.max_angular_rate = 0.5;
  const PurePursuit tracker{straight_path(10.0, 0.5), config};
  // Facing backwards: huge curvature demand.
  const auto cmd = tracker.track(Pose2{2.0, 1.0, kPi});
  EXPECT_LE(std::abs(cmd.twist.angular), 0.5 + 1e-12);
}

TEST(PurePursuitTest, ConvergesToStraightPathInClosedLoop) {
  // Closed loop with diff-drive kinematics: start offset, converge, arrive.
  PurePursuitConfig<double> config;
  config.cruise_speed = 1.0;
  config.goal_tolerance = 0.2;
  const PurePursuit tracker{straight_path(8.0, 0.25), config};
  estimation::DiffDriveOdometry odom{estimation::DiffDriveModel<double>{0.1, 0.5},
                                     Pose2{0.0, 1.0, 0.0}};
  const double dt = 0.02;
  bool arrived = false;
  for (int i = 0; i < 4000 && !arrived; ++i) {
    const auto cmd = tracker.track(odom.pose());
    arrived = cmd.goal_reached;
    // Invert diff-drive kinematics to wheel rates.
    const double v = cmd.twist.linear.x;
    const double w = cmd.twist.angular;
    const double left = (v - 0.25 * w) / 0.1;
    const double right = (v + 0.25 * w) / 0.1;
    odom.update(left, right, dt);
  }
  EXPECT_TRUE(arrived);
  // Cross-track error must have collapsed on the way to the goal.
  EXPECT_LT(std::abs(odom.pose().y()), 0.25);
}

}  // namespace
}  // namespace ars::core::control
