#include <gtest/gtest.h>

#include "ars_core/estimation/ekf.hpp"

namespace ars::core::estimation {
namespace {

using math::Matrix;
using math::Vector;

constexpr double kTol = 1e-12;

// --- 1D constant-position model: state [x], direct position measurement. ---

using Ekf1 = Ekf<double, 1>;

const auto kIdentityF1 = [](const Vector<double, 1>& x) { return x; };
const auto kIdentityJ1 = [](const Vector<double, 1>&) { return Matrix<double, 1, 1>::identity(); };

TEST(EkfTest, PredictInflatesCovariance) {
  Ekf1 ekf{Vector<double, 1>{{0.0}}, Matrix<double, 1, 1>{{1.0}}};
  ekf.predict(kIdentityF1, kIdentityJ1, Matrix<double, 1, 1>{{0.5}});
  EXPECT_NEAR(ekf.covariance()(0, 0), 1.5, kTol);
  EXPECT_NEAR(ekf.state()(0, 0), 0.0, kTol);
}

TEST(EkfTest, UpdateMatchesAnalyticKalmanGain) {
  // Prior N(0, 1), measurement z=2 with R=1 -> posterior mean 1, var 0.5.
  Ekf1 ekf{Vector<double, 1>{{0.0}}, Matrix<double, 1, 1>{{1.0}}};
  const bool ok =
      ekf.update(Vector<double, 1>{{2.0}}, kIdentityF1, kIdentityJ1, Matrix<double, 1, 1>{{1.0}});
  ASSERT_TRUE(ok);
  EXPECT_NEAR(ekf.state()(0, 0), 1.0, kTol);
  EXPECT_NEAR(ekf.covariance()(0, 0), 0.5, kTol);
}

TEST(EkfTest, RepeatedMeasurementsConvergeToTruth) {
  Ekf1 ekf{Vector<double, 1>{{10.0}}, Matrix<double, 1, 1>{{100.0}}};
  for (int i = 0; i < 50; ++i) {
    ekf.predict(kIdentityF1, kIdentityJ1, Matrix<double, 1, 1>{{0.0}});
    ekf.update(Vector<double, 1>{{3.0}}, kIdentityF1, kIdentityJ1, Matrix<double, 1, 1>{{0.1}});
  }
  EXPECT_NEAR(ekf.state()(0, 0), 3.0, 1e-3);
  EXPECT_LT(ekf.covariance()(0, 0), 0.01);
}

TEST(EkfTest, SingularInnovationRejectsUpdate) {
  Ekf1 ekf{Vector<double, 1>{{1.0}}, Matrix<double, 1, 1>{{0.0}}};
  const auto zero_jacobian = [](const Vector<double, 1>&) { return Matrix<double, 1, 1>{}; };
  const bool ok =
      ekf.update(Vector<double, 1>{{5.0}}, kIdentityF1, zero_jacobian, Matrix<double, 1, 1>{{0.0}});
  EXPECT_FALSE(ok);
  EXPECT_NEAR(ekf.state()(0, 0), 1.0, kTol);
}

// --- 2D constant-velocity model: state [pos, vel], position-only sensor. ---

using Ekf2 = Ekf<double, 2>;
constexpr double kDt = 0.1;

const auto kCvF = [](const Vector<double, 2>& x) {
  return Vector<double, 2>{{x(0, 0) + kDt * x(1, 0), x(1, 0)}};
};
const auto kCvJ = [](const Vector<double, 2>&) {
  return Matrix<double, 2, 2>{{1.0, kDt, 0.0, 1.0}};
};
const auto kPosH = [](const Vector<double, 2>& x) { return Vector<double, 1>{{x(0, 0)}}; };
const auto kPosJ = [](const Vector<double, 2>&) { return Matrix<double, 1, 2>{{1.0, 0.0}}; };

TEST(EkfTest, ConstantVelocityTracksAnalyticTrajectory) {
  // True motion: x(t) = 2t. Position-only measurements; filter must infer velocity.
  Ekf2 ekf{Vector<double, 2>{{0.0, 0.0}}, Matrix<double, 2, 2>{{10.0, 0.0, 0.0, 10.0}}};
  const Matrix<double, 2, 2> q{{1e-6, 0.0, 0.0, 1e-6}};
  const Matrix<double, 1, 1> r{{1e-4}};
  for (int step = 1; step <= 200; ++step) {
    ekf.predict(kCvF, kCvJ, q);
    const double true_pos = 2.0 * kDt * step;
    ekf.update(Vector<double, 1>{{true_pos}}, kPosH, kPosJ, r);
  }
  EXPECT_NEAR(ekf.state()(0, 0), 2.0 * kDt * 200, 1e-3);
  EXPECT_NEAR(ekf.state()(1, 0), 2.0, 1e-2);
}

TEST(EkfTest, CovarianceStaysSymmetric) {
  Ekf2 ekf{Vector<double, 2>{{0.0, 1.0}}, Matrix<double, 2, 2>{{5.0, 0.1, 0.1, 5.0}}};
  const Matrix<double, 2, 2> q{{0.01, 0.0, 0.0, 0.01}};
  const Matrix<double, 1, 1> r{{0.5}};
  for (int step = 0; step < 100; ++step) {
    ekf.predict(kCvF, kCvJ, q);
    ekf.update(Vector<double, 1>{{0.1 * step}}, kPosH, kPosJ, r);
    EXPECT_NEAR(ekf.covariance()(0, 1), ekf.covariance()(1, 0), kTol);
    EXPECT_GT(ekf.covariance()(0, 0), 0.0);
    EXPECT_GT(ekf.covariance()(1, 1), 0.0);
  }
}

}  // namespace
}  // namespace ars::core::estimation
