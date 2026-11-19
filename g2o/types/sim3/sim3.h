// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SIM_3
#define G2O_SIM_3

#include <g2o/types/slam3d/se3_ops.h>

#include <Eigen/Geometry>
#include <array>
#include <cassert>
#include <cmath>
#include <complex>

G2O_START_NAMESPACE
typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

namespace internal {

/**
 * Matrix \f$W = A\Omega + B\Omega^2 + C I\f$ that maps the translational
 * part of the tangent vector of a similarity to its translation. It is the
 * integral \f$\int_0^1 e^{\sigma s} \exp(s\Omega)\,ds\f$ of the rotation
 * \f$\Omega\f$ with the angle \f$\theta\f$ and the logarithm
 * \f$\sigma\f$ of the scale, whose coefficients are
 * \f[
 *   A = \int_0^1 e^{\sigma s} \frac{\sin \theta s}{\theta}\,ds, \quad
 *   B = \int_0^1 e^{\sigma s} \frac{1 - \cos \theta s}{\theta^2}\,ds,
 *   \quad
 *   C = \int_0^1 e^{\sigma s}\,ds.
 * \f]
 * Below an angle of one, the coefficients are series in \f$\theta^2\f$
 * over the moments \f$M_n = \int_0^1 s^n e^{\sigma s}\,ds\f$, whose closed
 * forms cancel for small angles and scales. Above it, they follow from
 * \f$(e^z - 1)/z\f$ with \f$z = \sigma + i\theta\f$.
 */
inline Eigen::Matrix3d sim3W(const Eigen::Vector3d& omega, double theta,
                             double sigma) {
  using std::exp;
  using std::expm1;
  using std::fpclassify;

  const double C = fpclassify(sigma) == FP_ZERO ? 1. : expm1(sigma) / sigma;
  double A = 0.;
  double B = 0.;
  if (theta < 1.) {
    // The terms decrease at least as 1/(2k)!, which falls below the precision
    // of a double after this many.
    constexpr int Terms = 10;
    constexpr int Moments = 2 * Terms + 2;
    // The backward recurrence M_{n-1} = (e^sigma - sigma M_n) / n starts
    // from the value e^sigma / (n + 1) that the moments approach for large n.
    // Its error reaches the coefficients only through the smallest terms.
    const double es = exp(sigma);
    double moment = es / (Moments + 1);
    std::array<double, Moments + 1> moments{};
    for (int n = Moments; n >= 0; --n) {
      moments[n] = moment;
      if (n > 0) {
        moment = (es - sigma * moment) / n;
      }
    }
    // The terms (-theta^2)^k M_{2k+1} / (2k+1)! of A and (-theta^2)^k
    // M_{2k+2} / (2k+2)! of B summed from the smallest.
    const double theta2 = theta * theta;
    std::array<double, Terms> weights{};
    double weight = 1.;
    for (int k = 0; k < Terms; ++k) {
      weights[k] = weight;
      weight *= -theta2 / ((2 * k + 2) * (2 * k + 3));
    }
    for (int k = Terms - 1; k >= 0; --k) {
      A += weights[k] * moments[2 * k + 1];
      B += weights[k] / (2 * k + 2) * moments[2 * k + 2];
    }
  } else {
    const std::complex<double> z(sigma, theta);
    const std::complex<double> F = (exp(z) - 1.) / z;
    A = F.imag() / theta;
    B = (C - F.real()) / (theta * theta);
  }

  const Eigen::Matrix3d Omega = skew(omega);
  return A * Omega + B * Omega * Omega + C * Eigen::Matrix3d::Identity();
}

}  // namespace internal

struct Sim3 {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  Eigen::Quaterniond r;
  Eigen::Vector3d t;
  double s;

 public:
  Sim3() {
    r.setIdentity();
    t.fill(0.);
    s = 1.;
  }

  Sim3(const Eigen::Quaterniond& r, const Eigen::Vector3d& t, double s)
      : r(r), t(t), s(s) {
    normalizeRotation();
  }

  Sim3(const Eigen::Matrix3d& R, const Eigen::Vector3d& t, double s)
      : r(Eigen::Quaterniond(R)), t(t), s(s) {
    normalizeRotation();
  }

  /**
   * Exponential of a tangent vector (omega, upsilon, sigma) of rotation,
   * translation and logarithm of the scale.
   */
  Sim3(const Vector7d& update) {
    const Eigen::Vector3d omega = update.head<3>();
    const Eigen::Vector3d upsilon = update.segment<3>(3);
    const double sigma = update[6];
    const double theta = omega.norm();

    s = std::exp(sigma);
    if (std::fpclassify(theta) == FP_ZERO) {
      r.setIdentity();
    } else {
      r = Eigen::AngleAxisd(theta, omega / theta);
    }
    t = internal::sim3W(omega, theta, sigma) * upsilon;
  }

  Eigen::Vector3d map(const Eigen::Vector3d& xyz) const {
    return s * (r * xyz) + t;
  }

  //! Tangent vector (omega, upsilon, sigma) whose exponential this is.
  Vector7d log() const {
    const Eigen::AngleAxisd rotation(r);
    const Eigen::Vector3d omega = rotation.angle() * rotation.axis();
    const double sigma = std::log(s);

    Vector7d res;
    res.head<3>() = omega;
    res.segment<3>(3) =
        internal::sim3W(omega, rotation.angle(), sigma).lu().solve(t);
    res[6] = sigma;
    return res;
  }

  Sim3 inverse() const {
    return Sim3(r.conjugate(), r.conjugate() * ((-1. / s) * t), 1. / s);
  }

  double operator[](int i) const {
    assert(i < 8);
    if (i < 4) {
      return r.coeffs()[i];
    }
    if (i < 7) {
      return t[i - 4];
    }
    return s;
  }

  double& operator[](int i) {
    assert(i < 8);
    if (i < 4) {
      return r.coeffs()[i];
    }
    if (i < 7) {
      return t[i - 4];
    }
    return s;
  }

  Sim3 operator*(const Sim3& other) const {
    Sim3 ret;
    ret.r = r * other.r;
    ret.t = s * (r * other.t) + t;
    ret.s = s * other.s;
    return ret;
  }

  Sim3& operator*=(const Sim3& other) {
    Sim3 ret = (*this) * other;
    *this = ret;
    return *this;
  }
  void normalizeRotation() {
    if (r.w() < 0) {
      r.coeffs() *= -1;
    }
    r.normalize();
  }
  inline const Eigen::Vector3d& translation() const { return t; }

  inline Eigen::Vector3d& translation() { return t; }

  inline const Eigen::Quaterniond& rotation() const { return r; }

  inline Eigen::Quaterniond& rotation() { return r; }

  inline const double& scale() const { return s; }

  inline double& scale() { return s; }
};

inline std::ostream& operator<<(std::ostream& out_str, const Sim3& sim3) {
  out_str << sim3.rotation().coeffs() << std::endl;
  out_str << sim3.translation() << std::endl;
  out_str << sim3.scale() << std::endl;

  return out_str;
}

G2O_END_NAMESPACE

#endif
