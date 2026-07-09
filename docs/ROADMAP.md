# Roadmap

Milestone-driven development. Each milestone ends with a tagged release,
green CI, and updated documentation.

## M1 — Core Foundations (in progress)

Pure C++20, no ROS. The mathematical backbone of the stack.

- [x] Repository scaffolding, tooling, CI (sanitized builds, format gate)
- [x] `geometry::Vec2` — constexpr 2D vector algebra
- [x] `geometry::Angle` — normalized angle type ([-π, π)), safe arithmetic
- [ ] `geometry::Pose2` — SE(2) rigid transform, compose/inverse/relative-to
- [ ] `geometry::Twist2` — planar velocity, body/world frame conversions

## M2 — State Estimation

- [ ] `estimation::Ekf` — generic EKF over concept-constrained models
- [ ] Differential-drive odometry motion model
- [ ] Unit tests against analytic ground truth trajectories

## M3 — Motion Planning

- [ ] `planning::OccupancyGrid` — value-semantic grid map
- [ ] `planning::AStar` — grid A* with pluggable heuristics
- [ ] `planning::PathSmoother` — gradient-based smoothing

## M4 — Trajectory Control

- [ ] `control::PidController` — anti-windup, derivative filtering
- [ ] `control::PurePursuit` — path tracking with adaptive lookahead

## M5 — ROS 2 Integration

- [ ] `ars_ros2` adapter layer: lifecycle nodes, message conversions
- [ ] colcon build alongside standalone CMake

## M6 — Simulation

- [ ] Gazebo Harmonic diff-drive AMR model
- [ ] Full SIL pipeline: sensors → estimation → planning → control

## M7 — Mission Execution

- [ ] Behavior-tree mission layer, waypoint missions, recovery behaviors
