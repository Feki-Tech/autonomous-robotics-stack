#pragma once

#include <concepts>
#include <cstddef>
#include <vector>

#include "ars_core/geometry/vec2.hpp"

namespace ars::core::planning {

/// Parameters for gradient-based path smoothing.
template <std::floating_point T = double>
struct SmootherConfig {
  T data_weight{0.2};    ///< pull toward the original path
  T smooth_weight{0.6};  ///< pull toward neighbor midpoints
  T tolerance{1e-9};     ///< stop when total update falls below this
  std::size_t max_iterations{1000};
};

/// Smooths a polyline by iterative gradient descent on
///   E = data_weight * |p - original|^2 + smooth_weight * |p_prev - 2p + p_next|^2
/// with both endpoints held fixed. The classic elastic-band smoother: removes
/// grid-path staircase artifacts while staying anchored to the planned route.
template <std::floating_point T = double>
[[nodiscard]] std::vector<geometry::Vec2<T>> smooth_path(const std::vector<geometry::Vec2<T>>& path,
                                                         const SmootherConfig<T>& config = {}) {
  if (path.size() < 3) {
    return path;
  }
  std::vector<geometry::Vec2<T>> smoothed = path;
  for (std::size_t iteration = 0; iteration < config.max_iterations; ++iteration) {
    T total_change{};
    for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
      const auto before = smoothed[i];
      const auto data_pull = (path[i] - smoothed[i]) * config.data_weight;
      const auto smooth_pull =
          (smoothed[i - 1] + smoothed[i + 1] - smoothed[i] * T{2}) * config.smooth_weight;
      smoothed[i] = smoothed[i] + data_pull + smooth_pull;
      total_change += (smoothed[i] - before).norm();
    }
    if (total_change < config.tolerance) {
      break;
    }
  }
  return smoothed;
}

}  // namespace ars::core::planning
