#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "ars_core/planning/path_smoother.hpp"

namespace ars::core::planning {
namespace {

using geometry::Vec2;
using Path = std::vector<Vec2<double>>;

double max_corner_angle(const Path& path) {
  double max_angle = 0.0;
  for (std::size_t i = 1; i + 1 < path.size(); ++i) {
    const auto a = path[i] - path[i - 1];
    const auto b = path[i + 1] - path[i];
    const double angle = std::abs(std::atan2(a.cross(b), a.dot(b)));
    max_angle = std::max(max_angle, angle);
  }
  return max_angle;
}

TEST(PathSmootherTest, ShortPathsPassThrough) {
  const Path empty{};
  EXPECT_TRUE(smooth_path(empty).empty());
  const Path two{{0.0, 0.0}, {1.0, 0.0}};
  EXPECT_EQ(smooth_path(two), two);
}

TEST(PathSmootherTest, EndpointsStayFixed) {
  const Path path{{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}, {3.0, 1.0}, {4.0, 0.0}};
  const auto smoothed = smooth_path(path);
  EXPECT_EQ(smoothed.front(), path.front());
  EXPECT_EQ(smoothed.back(), path.back());
}

TEST(PathSmootherTest, StraightLineIsFixedPoint) {
  Path path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back({static_cast<double>(i), 0.0});
  }
  const auto smoothed = smooth_path(path);
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_NEAR(smoothed[i].x, path[i].x, 1e-6);
    EXPECT_NEAR(smoothed[i].y, path[i].y, 1e-6);
  }
}

TEST(PathSmootherTest, ReducesStaircaseCorners) {
  // Grid staircase: alternating right/up unit moves (90-degree corners).
  Path staircase;
  double x = 0.0;
  double y = 0.0;
  staircase.push_back({x, y});
  for (int i = 0; i < 6; ++i) {
    staircase.push_back({++x, y});
    staircase.push_back({x, ++y});
  }
  const auto smoothed = smooth_path(staircase);
  EXPECT_LT(max_corner_angle(smoothed), max_corner_angle(staircase));
}

TEST(PathSmootherTest, StaysAnchoredToOriginal) {
  const Path path{{0.0, 0.0}, {1.0, 2.0}, {2.0, 0.0}, {3.0, 2.0}, {4.0, 0.0}};
  const auto smoothed = smooth_path(path);
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_LT((smoothed[i] - path[i]).norm(), 2.0);
  }
}

TEST(PathSmootherTest, ZeroSmoothWeightPreservesPath) {
  const Path path{{0.0, 0.0}, {1.0, 1.0}, {2.0, 0.0}};
  SmootherConfig<double> config;
  config.smooth_weight = 0.0;
  const auto smoothed = smooth_path(path, config);
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_NEAR(smoothed[i].x, path[i].x, 1e-9);
    EXPECT_NEAR(smoothed[i].y, path[i].y, 1e-9);
  }
}

}  // namespace
}  // namespace ars::core::planning
