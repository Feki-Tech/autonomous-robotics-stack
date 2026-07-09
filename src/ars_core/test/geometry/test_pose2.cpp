#include <gtest/gtest.h>

#include <numbers>

#include "ars_core/geometry/pose2.hpp"

namespace ars::core::geometry {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-12;

void ExpectPoseNear(const Pose2<>& actual, const Pose2<>& expected, double tol = kTol) {
  EXPECT_NEAR(actual.x(), expected.x(), tol);
  EXPECT_NEAR(actual.y(), expected.y(), tol);
  EXPECT_NEAR(actual.rotation().radians(), expected.rotation().radians(), tol);
}

TEST(Pose2Test, DefaultIsIdentity) {
  constexpr Pose2 p;
  EXPECT_DOUBLE_EQ(p.x(), 0.0);
  EXPECT_DOUBLE_EQ(p.y(), 0.0);
  EXPECT_DOUBLE_EQ(p.rotation().radians(), 0.0);
}

TEST(Pose2Test, TransformPureTranslation) {
  const Pose2 p{1.0, 2.0, 0.0};
  const auto q = p.transform({3.0, 4.0});
  EXPECT_NEAR(q.x, 4.0, kTol);
  EXPECT_NEAR(q.y, 6.0, kTol);
}

TEST(Pose2Test, TransformPureRotation) {
  const Pose2 p{0.0, 0.0, kPi / 2.0};
  const auto q = p.transform({1.0, 0.0});
  EXPECT_NEAR(q.x, 0.0, kTol);
  EXPECT_NEAR(q.y, 1.0, kTol);
}

TEST(Pose2Test, TransformRotationThenTranslation) {
  const Pose2 p{1.0, 1.0, kPi / 2.0};
  const auto q = p.transform({1.0, 0.0});
  EXPECT_NEAR(q.x, 1.0, kTol);
  EXPECT_NEAR(q.y, 2.0, kTol);
}

TEST(Pose2Test, ComposeWithIdentityIsNoop) {
  const Pose2 p{2.0, -1.0, 0.7};
  ExpectPoseNear(p.compose(Pose2{}), p);
  ExpectPoseNear(Pose2{}.compose(p), p);
}

TEST(Pose2Test, ComposeChainsTransforms) {
  const Pose2 a{1.0, 0.0, kPi / 2.0};
  const Pose2 b{1.0, 0.0, 0.0};
  ExpectPoseNear(a.compose(b), Pose2{1.0, 1.0, kPi / 2.0});
}

TEST(Pose2Test, ComposeWrapsRotation) {
  const Pose2 a{0.0, 0.0, kPi - 0.1};
  const Pose2 b{0.0, 0.0, 0.2};
  EXPECT_NEAR(a.compose(b).rotation().radians(), -kPi + 0.1, kTol);
}

TEST(Pose2Test, InverseComposesToIdentity) {
  const Pose2 p{3.0, -2.0, 1.1};
  ExpectPoseNear(p.compose(p.inverse()), Pose2{});
  ExpectPoseNear(p.inverse().compose(p), Pose2{});
}

TEST(Pose2Test, InverseUndoesTransform) {
  const Pose2 p{3.0, -2.0, 1.1};
  const Vec2 point{0.5, -1.5};
  const auto round_trip = p.inverse().transform(p.transform(point));
  EXPECT_NEAR(round_trip.x, point.x, kTol);
  EXPECT_NEAR(round_trip.y, point.y, kTol);
}

TEST(Pose2Test, RelativeToRecoversDelta) {
  const Pose2 base{1.0, 2.0, kPi / 4.0};
  const Pose2 delta{0.5, 0.0, 0.3};
  const auto target = base.compose(delta);
  ExpectPoseNear(target.relative_to(base), delta);
}

TEST(Pose2Test, RelativeToSelfIsIdentity) {
  const Pose2 p{-4.0, 7.0, 2.5};
  ExpectPoseNear(p.relative_to(p), Pose2{});
}

}  // namespace
}  // namespace ars::core::geometry
