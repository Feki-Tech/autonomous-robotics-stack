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
   detail confined to a thin adapter layer.
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
| `ars_ros2`       | Lifecycle nodes, executors, message adapters           |   ⚪   |
| `ars_simulation` | Gazebo worlds, diff-drive AMR model, sensor bridges    |   ⚪   |
| `ars_missions`   | Behavior-tree mission execution                        |   ⚪   |
| `ars_bringup`    | Launch files, parameters, system composition           |   ⚪   |

## Build & Test (core, no ROS required)

```bash
cd src/ars_core
cmake --preset dev          # Debug + ASan/UBSan + warnings-as-errors
cmake --build --preset dev
ctest --preset dev
```

## License

MIT — see [LICENSE](LICENSE).
