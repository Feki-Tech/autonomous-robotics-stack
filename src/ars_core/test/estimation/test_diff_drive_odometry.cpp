#include <gtest/gtest.h>

#include <numbers>

#include "ars_core/estimation/diff_drive_odometry.hpp"

namespace ars::core::estimation {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-9;

// 0.1 m wheels, 0.5 m track.
constexpr DiffDriveModel<double> kModel{0.1, 0.5};

TEST(DiffDriveModelTest, EqualRatesDriveStraight) {
  const auto twist = kModel.forward(10.0, 10.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, 1.0);
  EXPECT_DOUBLE_EQ(twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular, 0.0);
}

TEST(DiffDriveModelTest, OppositeRatesSpinInPlace) {
  const auto twist = kModel.forward(-5.0, 5.0);
  EXPECT_DOUBLE_EQ(twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(twist.angular, 2.0);
}

TEST(DiffDriveOdometryTest, StartsAtInitialPose) {
  const DiffDriveOdometry odom{kModel, geometry::Pose2{1.0, 2.0, 0.5}};
  EXPECT_DOUBLE_EQ(odom.pose().x(), 1.0);
  EXPECT_DOUBLE_EQ(odom.pose().y(), 2.0);
  EXPECT_DOUBLE_EQ(odom.pose().rotation().radians(), 0.5);
}

TEST(DiffDriveOdometryTest, StraightLineMotion) {
  DiffDriveOdometry odom{kModel};
  for (int i = 0; i < 100; ++i) {
    odom.update(10.0, 10.0, 0.01);  // 1 m/s for 1 s total
  }
  EXPECT_NEAR(odom.pose().x(), 1.0, kTol);
  EXPECT_NEAR(odom.pose().y(), 0.0, kTol);
  EXPECT_NEAR(odom.pose().rotation().radians(), 0.0, kTol);
}

TEST(DiffDriveOdometryTest, SpinInPlace) {
  DiffDriveOdometry odom{kModel};
  // w = 2 rad/s for pi/2 s -> quarter turn... use pi/4 s for w*t = pi/2.
  const double dt = kPi / 4.0 / 100.0;
  for (int i = 0; i < 100; ++i) {
    odom.update(-5.0, 5.0, dt);
  }
  EXPECT_NEAR(odom.pose().x(), 0.0, kTol);
  EXPECT_NEAR(odom.pose().y(), 0.0, kTol);
  EXPECT_NEAR(odom.pose().rotation().radians(), kPi / 2.0, kTol);
}

TEST(DiffDriveOdometryTest, FullCircleReturnsToStart) {
  DiffDriveOdometry odom{kModel};
  // v = 0.75 m/s, w = 1.0 rad/s -> circle of radius 0.75 m, period 2pi s.
  const double total_time = 2.0 * kPi;
  const int steps = 10000;
  for (int i = 0; i < steps; ++i) {
    odom.update(5.0, 10.0, total_time / steps);
  }
  EXPECT_NEAR(odom.pose().x(), 0.0, 1e-6);
  EXPECT_NEAR(odom.pose().y(), 0.0, 1e-6);
  EXPECT_NEAR(odom.pose().rotation().radians(), 0.0, 1e-6);
}

TEST(DiffDriveOdometryTest, ArcIntegrationIsExactPerStep) {
  // One large step around a quarter circle: exact arc model should land
  // exactly on the analytic endpoint even with dt = full quarter turn.
  DiffDriveOdometry odom{kModel};
  // v = 0.75, w = 1.0 -> radius 0.75. Quarter turn takes pi/2 s.
  odom.update(5.0, 10.0, kPi / 2.0);
  EXPECT_NEAR(odom.pose().x(), 0.75, kTol);
  EXPECT_NEAR(odom.pose().y(), 0.75, kTol);
  EXPECT_NEAR(odom.pose().rotation().radians(), kPi / 2.0, kTol);
}

TEST(DiffDriveOdometryTest, VelocityReflectsLastUpdate) {
  DiffDriveOdometry odom{kModel};
  odom.update(10.0, 10.0, 0.1);
  EXPECT_DOUBLE_EQ(odom.velocity().linear.x, 1.0);
  EXPECT_DOUBLE_EQ(odom.velocity().angular, 0.0);
}

TEST(DiffDriveOdometryTest, ResetClearsState) {
  DiffDriveOdometry odom{kModel};
  odom.update(10.0, 5.0, 1.0);
  odom.reset(geometry::Pose2{});
  EXPECT_DOUBLE_EQ(odom.pose().x(), 0.0);
  EXPECT_DOUBLE_EQ(odom.velocity().linear.x, 0.0);
}

}  // namespace
}  // namespace ars::core::estimation
