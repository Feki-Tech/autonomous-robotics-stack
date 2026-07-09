#include <gtest/gtest.h>

#include "ars_core/planning/occupancy_grid.hpp"

namespace ars::core::planning {
namespace {

using Grid = OccupancyGrid<double>;

TEST(OccupancyGridTest, ConstructsAllFree) {
  const Grid grid{10, 5, 0.1};
  EXPECT_EQ(grid.width(), 10u);
  EXPECT_EQ(grid.height(), 5u);
  for (std::int64_t y = 0; y < 5; ++y) {
    for (std::int64_t x = 0; x < 10; ++x) {
      EXPECT_TRUE(grid.is_free({x, y}));
    }
  }
}

TEST(OccupancyGridTest, SetAndReadBack) {
  Grid grid{4, 4, 1.0};
  grid.set({2, 3}, Grid::Cell::kOccupied);
  EXPECT_EQ(grid.at({2, 3}), Grid::Cell::kOccupied);
  EXPECT_FALSE(grid.is_free({2, 3}));
  EXPECT_TRUE(grid.is_free({3, 2}));
}

TEST(OccupancyGridTest, ContainsRejectsOutOfBounds) {
  const Grid grid{4, 4, 1.0};
  EXPECT_TRUE(grid.contains({0, 0}));
  EXPECT_TRUE(grid.contains({3, 3}));
  EXPECT_FALSE(grid.contains({4, 0}));
  EXPECT_FALSE(grid.contains({0, 4}));
  EXPECT_FALSE(grid.contains({-1, 0}));
}

TEST(OccupancyGridTest, OutOfBoundsIsNotFree) {
  const Grid grid{4, 4, 1.0};
  EXPECT_FALSE(grid.is_free({-1, 0}));
  EXPECT_FALSE(grid.is_free({0, 7}));
}

TEST(OccupancyGridTest, FillRectangleClipsToBounds) {
  Grid grid{4, 4, 1.0};
  grid.fill_rectangle({2, 2}, {10, 10});
  EXPECT_FALSE(grid.is_free({2, 2}));
  EXPECT_FALSE(grid.is_free({3, 3}));
  EXPECT_TRUE(grid.is_free({1, 1}));
}

TEST(OccupancyGridTest, CellToWorldReturnsCenter) {
  const Grid grid{10, 10, 0.5, {1.0, 2.0}};
  const auto p = grid.cell_to_world({0, 0});
  EXPECT_DOUBLE_EQ(p.x, 1.25);
  EXPECT_DOUBLE_EQ(p.y, 2.25);
  const auto q = grid.cell_to_world({3, 1});
  EXPECT_DOUBLE_EQ(q.x, 2.75);
  EXPECT_DOUBLE_EQ(q.y, 2.75);
}

TEST(OccupancyGridTest, WorldToCellRoundTrip) {
  const Grid grid{10, 10, 0.5, {1.0, 2.0}};
  for (std::int64_t y = 0; y < 10; ++y) {
    for (std::int64_t x = 0; x < 10; ++x) {
      const auto index = grid.world_to_cell(grid.cell_to_world({x, y}));
      ASSERT_TRUE(index.has_value());
      EXPECT_EQ(*index, (GridIndex{x, y}));
    }
  }
}

TEST(OccupancyGridTest, WorldToCellRejectsOutside) {
  const Grid grid{10, 10, 0.5, {1.0, 2.0}};
  EXPECT_FALSE(grid.world_to_cell({0.5, 2.5}).has_value());
  EXPECT_FALSE(grid.world_to_cell({6.5, 2.5}).has_value());
  EXPECT_FALSE(grid.world_to_cell({1.5, 7.5}).has_value());
}

TEST(OccupancyGridTest, ValueSemantics) {
  Grid a{4, 4, 1.0};
  Grid b = a;
  b.set({1, 1}, Grid::Cell::kOccupied);
  EXPECT_TRUE(a.is_free({1, 1}));
  EXPECT_FALSE(b.is_free({1, 1}));
  EXPECT_FALSE(a == b);
}

}  // namespace
}  // namespace ars::core::planning
