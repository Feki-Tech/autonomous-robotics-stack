#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <queue>
#include <vector>

#include "ars_core/planning/occupancy_grid.hpp"

namespace ars::core::planning {

/// A heuristic estimates remaining cost between two cells. It must never
/// overestimate the true cost (admissibility) for A* to stay optimal.
template <typename H, typename T>
concept Heuristic = requires(H h, GridIndex a, GridIndex b) {
  { h(a, b) } -> std::convertible_to<T>;
};

/// Octile distance: exact remaining cost on an 8-connected grid with unit
/// straight moves and sqrt(2) diagonal moves. Admissible and consistent.
template <std::floating_point T = double>
struct OctileDistance {
  constexpr T operator()(GridIndex a, GridIndex b) const {
    const T dx = static_cast<T>(a.x > b.x ? a.x - b.x : b.x - a.x);
    const T dy = static_cast<T>(a.y > b.y ? a.y - b.y : b.y - a.y);
    constexpr T kDiagonal = std::numbers::sqrt2_v<T> - T{2};
    return dx + dy + kDiagonal * (dx < dy ? dx : dy);
  }
};

/// Grid A* over an 8-connected occupancy grid.
///
/// Returns the optimal cell path from start to goal (inclusive), or nullopt
/// if either endpoint is blocked or no path exists. Diagonal moves are not
/// allowed to cut corners past occupied cells.
template <std::floating_point T = double, Heuristic<T> H = OctileDistance<T>>
[[nodiscard]] std::optional<std::vector<GridIndex>> find_path(const OccupancyGrid<T>& grid,
                                                              GridIndex start, GridIndex goal,
                                                              H heuristic = {}) {
  if (!grid.is_free(start) || !grid.is_free(goal)) {
    return std::nullopt;
  }

  const auto flatten = [&grid](GridIndex i) {
    return static_cast<std::size_t>(i.y) * grid.width() + static_cast<std::size_t>(i.x);
  };
  const std::size_t cell_count = grid.width() * grid.height();
  constexpr std::size_t kNoParent = static_cast<std::size_t>(-1);
  constexpr T kInfinity = std::numeric_limits<T>::infinity();

  std::vector<T> best_cost(cell_count, kInfinity);
  std::vector<std::size_t> parent(cell_count, kNoParent);
  std::vector<bool> closed(cell_count, false);

  struct QueueEntry {
    T priority;
    T cost;
    GridIndex index;
    bool operator>(const QueueEntry& rhs) const { return priority > rhs.priority; }
  };
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<>> open;

  best_cost[flatten(start)] = T{0};
  open.push({heuristic(start, goal), T{0}, start});

  while (!open.empty()) {
    const auto current = open.top();
    open.pop();
    const std::size_t current_flat = flatten(current.index);
    if (closed[current_flat]) {
      continue;
    }
    closed[current_flat] = true;

    if (current.index == goal) {
      std::vector<GridIndex> path;
      for (std::size_t i = current_flat; i != kNoParent; i = parent[i]) {
        path.push_back({static_cast<std::int64_t>(i % grid.width()),
                        static_cast<std::int64_t>(i / grid.width())});
      }
      std::ranges::reverse(path);
      return path;
    }

    for (std::int64_t dy = -1; dy <= 1; ++dy) {
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        const GridIndex next{current.index.x + dx, current.index.y + dy};
        if (!grid.is_free(next)) {
          continue;
        }
        const bool diagonal = dx != 0 && dy != 0;
        // Forbid squeezing diagonally between two occupied cells.
        if (diagonal && (!grid.is_free({current.index.x + dx, current.index.y}) ||
                         !grid.is_free({current.index.x, current.index.y + dy}))) {
          continue;
        }
        const T step = diagonal ? std::numbers::sqrt2_v<T> : T{1};
        const T new_cost = current.cost + step;
        const std::size_t next_flat = flatten(next);
        if (new_cost < best_cost[next_flat]) {
          best_cost[next_flat] = new_cost;
          parent[next_flat] = current_flat;
          open.push({new_cost + heuristic(next, goal), new_cost, next});
        }
      }
    }
  }
  return std::nullopt;
}

}  // namespace ars::core::planning
