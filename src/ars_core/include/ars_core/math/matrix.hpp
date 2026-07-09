#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <optional>

namespace ars::core::math {

/// Fixed-size, stack-allocated, constexpr-friendly matrix.
///
/// Sized for estimation and control state spaces (typically <= 6x6), where a
/// value type with no heap allocation beats a general linear algebra library.
template <std::floating_point T, std::size_t Rows, std::size_t Cols>
class Matrix {
 public:
  constexpr Matrix() = default;

  /// Row-major brace initialization: Matrix<double, 2, 2> m{{1, 2, 3, 4}}.
  constexpr explicit Matrix(const std::array<T, Rows * Cols>& values) : data_{values} {}

  [[nodiscard]] static constexpr Matrix zero() { return Matrix{}; }

  [[nodiscard]] static constexpr Matrix identity()
    requires(Rows == Cols)
  {
    Matrix m;
    for (std::size_t i = 0; i < Rows; ++i) {
      m(i, i) = T{1};
    }
    return m;
  }

  [[nodiscard]] constexpr T& operator()(std::size_t row, std::size_t col) {
    return data_[row * Cols + col];
  }
  [[nodiscard]] constexpr const T& operator()(std::size_t row, std::size_t col) const {
    return data_[row * Cols + col];
  }

  static constexpr std::size_t rows() { return Rows; }
  static constexpr std::size_t cols() { return Cols; }

  constexpr Matrix operator+(const Matrix& rhs) const {
    Matrix out;
    for (std::size_t i = 0; i < Rows * Cols; ++i) {
      out.data_[i] = data_[i] + rhs.data_[i];
    }
    return out;
  }

  constexpr Matrix operator-(const Matrix& rhs) const {
    Matrix out;
    for (std::size_t i = 0; i < Rows * Cols; ++i) {
      out.data_[i] = data_[i] - rhs.data_[i];
    }
    return out;
  }

  constexpr Matrix operator*(T scalar) const {
    Matrix out;
    for (std::size_t i = 0; i < Rows * Cols; ++i) {
      out.data_[i] = data_[i] * scalar;
    }
    return out;
  }

  template <std::size_t OtherCols>
  constexpr Matrix<T, Rows, OtherCols> operator*(const Matrix<T, Cols, OtherCols>& rhs) const {
    Matrix<T, Rows, OtherCols> out;
    for (std::size_t i = 0; i < Rows; ++i) {
      for (std::size_t k = 0; k < Cols; ++k) {
        const T lhs_ik = (*this)(i, k);
        for (std::size_t j = 0; j < OtherCols; ++j) {
          out(i, j) += lhs_ik * rhs(k, j);
        }
      }
    }
    return out;
  }

  [[nodiscard]] constexpr Matrix<T, Cols, Rows> transpose() const {
    Matrix<T, Cols, Rows> out;
    for (std::size_t i = 0; i < Rows; ++i) {
      for (std::size_t j = 0; j < Cols; ++j) {
        out(j, i) = (*this)(i, j);
      }
    }
    return out;
  }

  /// Inverse by Gauss-Jordan elimination with partial pivoting.
  /// Returns std::nullopt for (numerically) singular matrices.
  [[nodiscard]] constexpr std::optional<Matrix> inverse() const
    requires(Rows == Cols)
  {
    Matrix a = *this;
    Matrix inv = identity();
    for (std::size_t col = 0; col < Cols; ++col) {
      std::size_t pivot = col;
      for (std::size_t row = col + 1; row < Rows; ++row) {
        if (abs_value(a(row, col)) > abs_value(a(pivot, col))) {
          pivot = row;
        }
      }
      if (abs_value(a(pivot, col)) <= T{0}) {
        return std::nullopt;
      }
      if (pivot != col) {
        swap_rows(a, pivot, col);
        swap_rows(inv, pivot, col);
      }
      const T scale = T{1} / a(col, col);
      for (std::size_t j = 0; j < Cols; ++j) {
        a(col, j) *= scale;
        inv(col, j) *= scale;
      }
      for (std::size_t row = 0; row < Rows; ++row) {
        if (row == col) {
          continue;
        }
        const T factor = a(row, col);
        for (std::size_t j = 0; j < Cols; ++j) {
          a(row, j) -= factor * a(col, j);
          inv(row, j) -= factor * inv(col, j);
        }
      }
    }
    return inv;
  }

  constexpr bool operator==(const Matrix&) const = default;

 private:
  static constexpr T abs_value(T v) { return v < T{0} ? -v : v; }

  static constexpr void swap_rows(Matrix& m, std::size_t a, std::size_t b) {
    for (std::size_t j = 0; j < Cols; ++j) {
      const T tmp = m(a, j);
      m(a, j) = m(b, j);
      m(b, j) = tmp;
    }
  }

  std::array<T, Rows * Cols> data_{};
};

/// Column vector convenience alias.
template <std::floating_point T, std::size_t N>
using Vector = Matrix<T, N, 1>;

}  // namespace ars::core::math
