#pragma once

#include <concepts>
#include <cstddef>
#include <optional>

#include "ars_core/math/matrix.hpp"

namespace ars::core::estimation {

/// A callable mapping a state vector to a vector of dimension Out.
template <typename F, typename T, std::size_t N, std::size_t Out>
concept VectorFunction = requires(F f, const math::Vector<T, N>& x) {
  { f(x) } -> std::convertible_to<math::Vector<T, Out>>;
};

/// A callable producing the Out x N Jacobian of such a function at x.
template <typename F, typename T, std::size_t N, std::size_t Out>
concept JacobianFunction = requires(F f, const math::Vector<T, N>& x) {
  { f(x) } -> std::convertible_to<math::Matrix<T, Out, N>>;
};

/// Extended Kalman filter over an N-dimensional state.
///
/// Model-agnostic: process and measurement models are supplied per call as
/// concept-constrained callables (function + Jacobian), so one filter type
/// serves any vehicle or sensor without inheritance or virtual dispatch.
template <std::floating_point T, std::size_t N>
class Ekf {
 public:
  using State = math::Vector<T, N>;
  using Covariance = math::Matrix<T, N, N>;

  constexpr Ekf(State initial_state, Covariance initial_covariance)
      : state_{initial_state}, covariance_{initial_covariance} {}

  [[nodiscard]] constexpr const State& state() const { return state_; }
  [[nodiscard]] constexpr const Covariance& covariance() const { return covariance_; }

  /// Propagates the state through the process model f with Jacobian jf and
  /// additive process noise q.
  template <VectorFunction<T, N, N> F, JacobianFunction<T, N, N> Jf>
  constexpr void predict(F&& f, Jf&& jf, const Covariance& q) {
    const auto jacobian = jf(state_);
    state_ = f(state_);
    covariance_ = jacobian * covariance_ * jacobian.transpose() + q;
  }

  /// Fuses a measurement z through model h with Jacobian jh and measurement
  /// noise r. Returns false (leaving the state untouched) if the innovation
  /// covariance is singular.
  template <std::size_t M, VectorFunction<T, N, M> H, JacobianFunction<T, N, M> Jh>
  constexpr bool update(const math::Vector<T, M>& z, H&& h, Jh&& jh,
                        const math::Matrix<T, M, M>& r) {
    const auto jacobian = jh(state_);
    const math::Vector<T, M> innovation = z - h(state_);
    const math::Matrix<T, M, M> s = jacobian * covariance_ * jacobian.transpose() + r;
    const auto s_inv = s.inverse();
    if (!s_inv.has_value()) {
      return false;
    }
    const math::Matrix<T, N, M> gain = covariance_ * jacobian.transpose() * *s_inv;
    state_ = state_ + gain * innovation;
    // Joseph-form update keeps the covariance symmetric positive semi-definite.
    const auto identity_minus_kh = Covariance::identity() - gain * jacobian;
    covariance_ = identity_minus_kh * covariance_ * identity_minus_kh.transpose() +
                  gain * r * gain.transpose();
    return true;
  }

 private:
  State state_;
  Covariance covariance_;
};

}  // namespace ars::core::estimation
