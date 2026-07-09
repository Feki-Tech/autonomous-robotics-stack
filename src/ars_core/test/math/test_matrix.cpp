#include <gtest/gtest.h>

#include "ars_core/math/matrix.hpp"

namespace ars::core::math {
namespace {

constexpr double kTol = 1e-12;

using Mat2 = Matrix<double, 2, 2>;
using Mat3 = Matrix<double, 3, 3>;

TEST(MatrixTest, DefaultIsZero) {
  constexpr Mat2 m;
  for (std::size_t i = 0; i < 2; ++i) {
    for (std::size_t j = 0; j < 2; ++j) {
      EXPECT_DOUBLE_EQ(m(i, j), 0.0);
    }
  }
}

TEST(MatrixTest, IdentityHasOnesOnDiagonal) {
  constexpr auto id = Mat3::identity();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_DOUBLE_EQ(id(i, j), i == j ? 1.0 : 0.0);
    }
  }
}

TEST(MatrixTest, AdditionAndSubtraction) {
  constexpr Mat2 a{{1.0, 2.0, 3.0, 4.0}};
  constexpr Mat2 b{{4.0, 3.0, 2.0, 1.0}};
  constexpr auto sum = a + b;
  EXPECT_DOUBLE_EQ(sum(0, 0), 5.0);
  EXPECT_DOUBLE_EQ(sum(1, 1), 5.0);
  constexpr auto diff = a - b;
  EXPECT_DOUBLE_EQ(diff(0, 0), -3.0);
  EXPECT_DOUBLE_EQ(diff(1, 1), 3.0);
}

TEST(MatrixTest, ScalarMultiplication) {
  constexpr Mat2 a{{1.0, -2.0, 0.5, 4.0}};
  constexpr auto scaled = a * 2.0;
  EXPECT_DOUBLE_EQ(scaled(0, 1), -4.0);
  EXPECT_DOUBLE_EQ(scaled(1, 0), 1.0);
}

TEST(MatrixTest, MatrixProduct) {
  constexpr Mat2 a{{1.0, 2.0, 3.0, 4.0}};
  constexpr Mat2 b{{5.0, 6.0, 7.0, 8.0}};
  constexpr auto p = a * b;
  EXPECT_DOUBLE_EQ(p(0, 0), 19.0);
  EXPECT_DOUBLE_EQ(p(0, 1), 22.0);
  EXPECT_DOUBLE_EQ(p(1, 0), 43.0);
  EXPECT_DOUBLE_EQ(p(1, 1), 50.0);
}

TEST(MatrixTest, NonSquareProductDimensions) {
  constexpr Matrix<double, 2, 3> a{{1.0, 0.0, 2.0, 0.0, 1.0, -1.0}};
  constexpr Vector<double, 3> v{{1.0, 2.0, 3.0}};
  constexpr auto p = a * v;
  static_assert(p.rows() == 2 && p.cols() == 1);
  EXPECT_DOUBLE_EQ(p(0, 0), 7.0);
  EXPECT_DOUBLE_EQ(p(1, 0), -1.0);
}

TEST(MatrixTest, Transpose) {
  constexpr Matrix<double, 2, 3> a{{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}};
  constexpr auto at = a.transpose();
  static_assert(at.rows() == 3 && at.cols() == 2);
  EXPECT_DOUBLE_EQ(at(0, 1), 4.0);
  EXPECT_DOUBLE_EQ(at(2, 0), 3.0);
}

TEST(MatrixTest, InverseTimesSelfIsIdentity) {
  constexpr Mat3 a{{2.0, 0.0, 1.0, 1.0, 3.0, 0.0, 0.0, 1.0, 4.0}};
  const auto inv = a.inverse();
  ASSERT_TRUE(inv.has_value());
  const auto id = a * *inv;
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) {
      EXPECT_NEAR(id(i, j), i == j ? 1.0 : 0.0, kTol);
    }
  }
}

TEST(MatrixTest, InverseRequiresPivoting) {
  // Zero on the leading diagonal forces a row swap.
  constexpr Mat2 a{{0.0, 1.0, 1.0, 0.0}};
  const auto inv = a.inverse();
  ASSERT_TRUE(inv.has_value());
  EXPECT_NEAR((*inv)(0, 1), 1.0, kTol);
  EXPECT_NEAR((*inv)(1, 0), 1.0, kTol);
}

TEST(MatrixTest, SingularMatrixHasNoInverse) {
  constexpr Mat2 a{{1.0, 2.0, 2.0, 4.0}};
  EXPECT_FALSE(a.inverse().has_value());
}

TEST(MatrixTest, ConstexprEvaluation) {
  constexpr Mat2 a{{1.0, 2.0, 3.0, 4.0}};
  constexpr auto b = (a + a) * a.transpose();
  static_assert(b(0, 0) == 10.0);
  SUCCEED();
}

}  // namespace
}  // namespace ars::core::math
