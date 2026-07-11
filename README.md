# Autonomous Robotics Stack

[![CI](https://github.com/Feki-Tech/autonomous-robotics-stack/actions/workflows/ci.yml/badge.svg)](https://github.com/Feki-Tech/autonomous-robotics-stack/actions/workflows/ci.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![ROS 2](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)](https://docs.ros.org/en/jazzy/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)

A production-grade software stack for autonomous mobile robots (AMR), built on modern
C++20 and ROS 2, validated software-in-the-loop in Gazebo.

The stack covers the full autonomy pipeline — state estimation, mapping, motion
planning, trajectory control, and mission execution — engineered with production
quality gates: warnings-as-errors, sanitized test suites in CI, static analysis,
and architecture decision records.

## Design Philosophy

1. **ROS-agnostic core.** All domain logic (geometry, estimation, planning, control)
   lives in pure C++20 libraries with zero ROS dependencies. ROS 2 is an integration
   detail confined to a thin adapter layer ([ADR-0002](docs/adr/0002-ros-agnostic-core.md)).
2. **Compile-time correctness first.** `constexpr` where possible, concept-constrained
   templates, RAII everywhere, no owning raw pointers.
3. **Tests are the specification.** Core components ship with exhaustive unit tests,
   executed under AddressSanitizer and UBSan in CI.
4. **Deterministic simulation before hardware.** Every feature is proven in Gazebo
   before any hardware bring-up.

## Architecture

Dependency rule: **arrows only point downward.** Message types are converted at the
adapter boundary; core libraries never include ROS headers.

## Packages

| Package          | Description                                            | Status |
| ---------------- | ------------------------------------------------------ | :----: |
| `ars_core`       | Geometry, estimation, planning, control (pure C++20)   |   🟢   |
| `ars_ros2`       | Lifecycle nodes, executors, message adapters           |   🟢   |
| `ars_sim`        | Gazebo worlds, diff-drive AMR model, sensor bridges    |   🟢   |
| `ars_missions`   | Behavior-tree mission execution (in core + ros2)       |   🟢   |
| `ars_bringup`    | Launch files, parameters, system composition           |   ⚪   |

See the [Roadmap](docs/ROADMAP.md).

## Build & Test (core, no ROS required)

```bash
cd src/ars_core
cmake --preset dev          # Debug + ASan/UBSan + warnings-as-errors
cmake --build --preset dev
ctest --preset dev
```

## Simulation (ROS 2 Jazzy + Gazebo Harmonic)

```bash
colcon build --packages-select ars_ros2 ars_sim
source install/setup.bash
ros2 launch ars_sim sil.launch.py        # headless; add headless:=false for the GUI
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    '{pose: {position: {x: 2.0, y: -1.0}}}'
```

The robot plans a path with A*, tracks it with pure pursuit, and localizes with
its own wheel odometry — Gazebo's ground truth is never consumed.

Add `mission:=true` to run an autonomous waypoint mission instead: a
behavior-tree mission node dispatches each waypoint, retries on stalls, and
reports completion.

## Run everything in Docker

No local ROS or Gazebo installation required — only Docker.

```bash
# Interactive dev shell (repository mounted at /ws): build, test, colcon — all inside.
docker compose run --rm dev

# Headless SIL simulation, fully self-contained image:
docker compose up sil
docker compose exec sil ros2 topic pub --once /goal_pose \
    geometry_msgs/msg/PoseStamped '{pose: {position: {x: 2.0, y: -1.0}}}'
```

The `dev` image carries the full toolchain (ROS 2 Jazzy, Gazebo Harmonic,
clang-format/-tidy, gdb); the `sil` image bakes a release build of the stack
and launches the simulation pipeline by default.

## License

MIT — see [LICENSE](LICENSE).
