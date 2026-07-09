#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "ars_core/geometry/vec2.hpp"

namespace ars::core::planning {

/// Integer cell coordinates in a grid map.
struct GridIndex {
  std::int64_t x{};
  std::int64_t y{};

  constexpr bool operator==(const GridIndex&) const = default;
};

/// A value-semantic 2D occupancy grid.
///
/// Cells hold an occupancy state; the grid maps between world coordinates
/// (meters, origin at the grid's lower-left corner) and cell indices. Copy,
/// move, and comparison behave like any regular value type.
template <std::floating_point T = double>
class OccupancyGrid {
 public:
  enum class Cell : std::uint8_t { kFree = 0, kOccupied = 1 };

  OccupancyGrid(std::size_t width, std::size_t height, T resolution, geometry::Vec2<T> origin = {})
      : width_{width},
        height_{height},
        resolution_{resolution},
        origin_{origin},
        cells_(width * height, Cell::kFree) {}

  [[nodiscard]] std::size_t width() const { return width_; }
  [[nodiscard]] std::size_t height() const { return height_; }
  [[nodiscard]] T resolution() const { return resolution_; }
  [[nodiscard]] const geometry::Vec2<T>& origin() const { return origin_; }

  [[nodiscard]] bool contains(GridIndex index) const {
    return index.x >= 0 && index.y >= 0 && static_cast<std::size_t>(index.x) < width_ &&
           static_cast<std::size_t>(index.y) < height_;
  }

  [[nodiscard]] Cell at(GridIndex index) const { return cells_[flatten(index)]; }
  void set(GridIndex index, Cell value) { cells_[flatten(index)] = value; }

  [[nodiscard]] bool is_free(GridIndex index) const {
    return contains(index) && at(index) == Cell::kFree;
  }

  /// Marks every cell in the inclusive index rectangle as occupied.
  void fill_rectangle(GridIndex min_corner, GridIndex max_corner, Cell value = Cell::kOccupied) {
    for (std::int64_t y = min_corner.y; y <= max_corner.y; ++y) {
      for (std::int64_t x = min_corner.x; x <= max_corner.x; ++x) {
        if (contains({x, y})) {
          set({x, y}, value);
        }
      }
    }
  }

  /// World position of a cell's center.
  [[nodiscard]] geometry::Vec2<T> cell_to_world(GridIndex index) const {
    return {origin_.x + (static_cast<T>(index.x) + T{0.5}) * resolution_,
            origin_.y + (static_cast<T>(index.y) + T{0.5}) * resolution_};
  }

  /// Cell containing a world position, or nullopt if outside the grid.
  [[nodiscard]] std::optional<GridIndex> world_to_cell(geometry::Vec2<T> position) const {
    const T gx = (position.x - origin_.x) / resolution_;
    const T gy = (position.y - origin_.y) / resolution_;
    if (gx < T{0} || gy < T{0}) {
      return std::nullopt;
    }
    const GridIndex index{static_cast<std::int64_t>(gx), static_cast<std::int64_t>(gy)};
    if (!contains(index)) {
      return std::nullopt;
    }
    return index;
  }

  bool operator==(const OccupancyGrid&) const = default;

 private:
  [[nodiscard]] std::size_t flatten(GridIndex index) const {
    return static_cast<std::size_t>(index.y) * width_ + static_cast<std::size_t>(index.x);
  }

  std::size_t width_;
  std::size_t height_;
  T resolution_;
  geometry::Vec2<T> origin_;
  std::vector<Cell> cells_;
};

}  // namespace ars::core::planning
