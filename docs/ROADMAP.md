# Roadmap

Milestone-driven development. Each milestone ends with a tagged release,
green CI, and updated documentation.

## M1 — Core Foundations (complete — v0.1.0)

Pure C++20, no ROS. The mathematical backbone of the stack.

- [x] Repository scaffolding, tooling, CI (sanitized builds, format gate)
- [x] `geometry::Vec2` — constexpr 2D vector algebra
- [x] `geometry::Angle` — normalized angle type ([-π, π)), safe arithmetic
- [x] `geometry::Pose2` — SE(2) rigid transform, compose/inverse/relative-to
- [x] `geometry::Twist2` — planar velocity, body/world frame conversions

## M2 — State Estimation (complete — v0.2.0)

- [x] `math::Matrix` — fixed-size constexpr matrix algebra (no external deps)
- [x] `estimation::DiffDriveOdometry` — wheel-speed dead reckoning
- [x] `estimation::Ekf` — generic EKF over concept-constrained models
- [x] Unit tests against analytic ground truth trajectories

## M3 — Motion Planning (complete — v0.3.0)

- [x] `planning::OccupancyGrid` — value-semantic grid map
- [x] `planning::AStar` — grid A* with pluggable heuristics
- [x] `planning::PathSmoother` — gradient-based smoothing

## M4 — Trajectory Control (complete — v0.4.0)

- [x] `control::PidController` — anti-windup, derivative filtering
- [x] `control::PurePursuit` — path tracking with adaptive lookahead

## M5 — ROS 2 Integration (complete — v0.5.0)

- [x] `ars_ros2` adapter layer: lifecycle nodes, message conversions
- [x] colcon build alongside standalone CMake

## M6 — Simulation (complete — v0.6.0)

- [x] Gazebo Harmonic diff-drive AMR model
- [x] Full SIL pipeline: sensors → estimation → planning → control

## M7 — Mission Execution

- [ ] Behavior-tree mission layer, waypoint missions, recovery behaviors
