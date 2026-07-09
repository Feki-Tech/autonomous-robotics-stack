# ADR-0002: ROS-agnostic core libraries

## Status

Accepted

## Context

Robotics stacks commonly entangle domain logic with ROS types (`geometry_msgs`,
`nav_msgs`, node handles), which makes the logic untestable without a ROS
installation, couples it to a specific middleware release, and slows CI.

## Decision

All domain logic (geometry, estimation, planning, control) lives in `ars_core`,
a pure C++20 library with **zero ROS dependencies**:

- `ars_core` builds standalone with CMake presets; CI needs only a compiler.
- ROS 2 is confined to a thin adapter layer (`ars_ros2`) that converts messages
  at the boundary and hosts lifecycle nodes.
- Core headers never include ROS headers; the dependency arrow points only
  from adapters to core.

## Consequences

- Core algorithms are unit-testable in milliseconds under ASan/UBSan, no
  ROS install or colcon workspace required.
- Message conversion code is explicit and centralized in the adapter layer.
- Slight duplication: core defines its own math types instead of reusing
  ROS message structs — an intentional trade for isolation and constexpr-ability.
