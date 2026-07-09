#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "ars_core/planning/astar.hpp"

namespace ars::core::planning {
namespace {

using Grid = OccupancyGrid<double>;

double path_length(const std::vector<GridIndex>& path) {
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const auto dx = static_cast<double>(path[i].x - path[i - 1].x);
    const auto dy = static_cast<double>(path[i].y - path[i - 1].y);
    length += std::hypot(dx, dy);
  }
  return length;
}

TEST(AStarTest, TrivialStartEqualsGoal) {
  const Grid grid{5, 5, 1.0};
  const auto path = find_path(grid, {2, 2}, {2, 2});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->size(), 1u);
}

TEST(AStarTest, StraightLineOnEmptyGrid) {
  const Grid grid{10, 10, 1.0};
  const auto path = find_path(grid, {0, 0}, {9, 0});
  ASSERT_TRUE(path.has_value());
  EXPECT_EQ(path->front(), (GridIndex{0, 0}));
  EXPECT_EQ(path->back(), (GridIndex{9, 0}));
  EXPECT_NEAR(path_length(*path), 9.0, 1e-12);
}

TEST(AStarTest, DiagonalIsOptimal) {
  const Grid grid{10, 10, 1.0};
  const auto path = find_path(grid, {0, 0}, {9, 9});
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), 9.0 * std::numbers::sqrt2, 1e-12);
}

TEST(AStarTest, RoutesAroundWall) {
  Grid grid{10, 10, 1.0};
  // Vertical wall at x=5 with a gap at y=9.
  grid.fill_rectangle({5, 0}, {5, 8});
  const auto path = find_path(grid, {0, 5}, {9, 5});
  ASSERT_TRUE(path.has_value());
  for (const auto& cell : *path) {
    EXPECT_TRUE(grid.is_free(cell));
  }
  // Must detour through the gap: longer than the straight 9-cell run.
  EXPECT_GT(path_length(*path), 9.0);
}

TEST(AStarTest, NoPathThroughFullWall) {
  Grid grid{10, 10, 1.0};
  grid.fill_rectangle({5, 0}, {5, 9});
  EXPECT_FALSE(find_path(grid, {0, 5}, {9, 5}).has_value());
}

TEST(AStarTest, BlockedEndpointsFail) {
  Grid grid{5, 5, 1.0};
  grid.set({0, 0}, Grid::Cell::kOccupied);
  EXPECT_FALSE(find_path(grid, {0, 0}, {4, 4}).has_value());
  EXPECT_FALSE(find_path(grid, {4, 4}, {0, 0}).has_value());
}

TEST(AStarTest, NoDiagonalCornerCutting) {
  Grid grid{3, 3, 1.0};
  // Occupied cells force any 0,0 -> 1,1 move to cut a corner.
  grid.set({1, 0}, Grid::Cell::kOccupied);
  grid.set({0, 1}, Grid::Cell::kOccupied);
  EXPECT_FALSE(find_path(grid, {0, 0}, {2, 2}).has_value());
}

TEST(AStarTest, PathCellsAreAdjacent) {
  Grid grid{20, 20, 1.0};
  grid.fill_rectangle({4, 4}, {15, 15});
  const auto path = find_path(grid, {0, 0}, {19, 19});
  ASSERT_TRUE(path.has_value());
  for (std::size_t i = 1; i < path->size(); ++i) {
    EXPECT_LE(std::abs((*path)[i].x - (*path)[i - 1].x), 1);
    EXPECT_LE(std::abs((*path)[i].y - (*path)[i - 1].y), 1);
  }
}

TEST(AStarTest, CustomHeuristicIsUsed) {
  const Grid grid{10, 10, 1.0};
  // Zero heuristic degrades A* to Dijkstra but must stay optimal.
  const auto dijkstra = [](GridIndex, GridIndex) { return 0.0; };
  const auto path = find_path(grid, {0, 0}, {9, 9}, dijkstra);
  ASSERT_TRUE(path.has_value());
  EXPECT_NEAR(path_length(*path), 9.0 * std::numbers::sqrt2, 1e-12);
}

}  // namespace
}  // namespace ars::core::planning
