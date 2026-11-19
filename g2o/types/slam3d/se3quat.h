// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2015-2025 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SE3QUAT_H_
#define G2O_SE3QUAT_H_

#include <g2o/core/eigen_types.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/se3_ops.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cassert>
#include <cmath>

G2O_START_NAMESPACE

namespace internal {

/**
 * Matrix \f$V = I + A\Omega + B\Omega^2\f$ that maps the translational part
 * of the tangent vector of a pose to its translation, with
 * \f$A = (1 - \cos\theta)/\theta^2 = 2\sin^2(\theta/2)/\theta^2\f$ and
 * \f$B = (\theta - \sin\theta)/\theta^3\f$. The difference of B cancels
 * for small angles, where its series replaces it.
 */
inline Matrix3D se3V(const Vector3D& omega, double theta) {
  using std::sin;
  // Below this angle, the series of B up to the sixth power is exact to the
  // precision of a double.
  constexpr double SeriesAngle = 0.1;
  const double theta2 = theta * theta;
  double A = 0.5;
  double B = 1. / 6.;
  if (std::fpclassify(theta) != FP_ZERO) {
    const double half = sin(theta / 2.) / theta;
    A = 2. * half * half;
  }
  if (theta < SeriesAngle) {
    B = 1. / 6. - theta2 / 120. + theta2 * theta2 / 5040. -
        theta2 * theta2 * theta2 / 362880.;
  } else {
    B = (theta - sin(theta)) / (theta2 * theta);
  }
  const Matrix3D Omega = skew(omega);
  return Matrix3D::Identity() + A * Omega + B * Omega * Omega;
}

}  // namespace internal

typedef Eigen::Matrix<double, 6, 1, Eigen::ColMajor> Vector6d;
typedef Eigen::Matrix<double, 7, 1, Eigen::ColMajor> Vector7d;

class G2O_TYPES_SLAM3D_API SE3Quat {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

 protected:
  Eigen::Quaterniond _r;
  Vector3D _t;

 public:
  SE3Quat() {
    _r.setIdentity();
    _t.setZero();
  }

  SE3Quat(const Matrix3D& R, const Vector3D& t)
      : _r(Eigen::Quaterniond(R)), _t(t) {
    normalizeRotation();
  }

  SE3Quat(const Eigen::Quaterniond& q, const Vector3D& t) : _r(q), _t(t) {
    normalizeRotation();
  }

  /**
   * templaized constructor which allows v to be an arbitrary Eigen Vector type,
   * e.g., Vector6d or Map<Vector6d>
   */
  template <typename Derived>
  explicit SE3Quat(const Eigen::MatrixBase<Derived>& v) {
    assert((v.size() == 6 || v.size() == 7) &&
           "Vector dimension does not match");
    if (v.size() == 6) {
      for (int i = 0; i < 3; i++) {
        _t[i] = v[i];
        _r.coeffs()(i) = v[i + 3];
      }
      _r.w() = 0.;  // recover the positive w
      if (_r.norm() > 1.) {
        _r.normalize();
      } else {
        double w2 = 1. - _r.squaredNorm();
        _r.w() = (w2 < 0.) ? 0. : sqrt(w2);
      }
    } else if (v.size() == 7) {
      int idx = 0;
      for (int i = 0; i < 3; ++i, ++idx) _t(i) = v(idx);
      for (int i = 0; i < 4; ++i, ++idx) _r.coeffs()(i) = v(idx);
      normalizeRotation();
    }
  }

  inline const Vector3D& translation() const { return _t; }

  inline void setTranslation(const Vector3D& t_) { _t = t_; }

  inline const Eigen::Quaterniond& rotation() const { return _r; }

  void setRotation(const Eigen::Quaterniond& r_) { _r = r_; }

  inline SE3Quat operator*(const SE3Quat& tr2) const {
    SE3Quat result(*this);
    result._t += _r * tr2._t;
    result._r *= tr2._r;
    result.normalizeRotation();
    return result;
  }

  inline SE3Quat& operator*=(const SE3Quat& tr2) {
    _t += _r * tr2._t;
    _r *= tr2._r;
    normalizeRotation();
    return *this;
  }

  inline Vector3D operator*(const Vector3D& v) const { return _t + _r * v; }

  inline SE3Quat inverse() const {
    SE3Quat ret;
    ret._r = _r.conjugate();
    ret._t = ret._r * (_t * -1.);
    return ret;
  }

  inline double operator[](int i) const {
    assert(i < 7);
    if (i < 3) return _t[i];
    return _r.coeffs()[i - 3];
  }

  inline Vector7d toVector() const {
    Vector7d v;
    v[0] = _t(0);
    v[1] = _t(1);
    v[2] = _t(2);
    v[3] = _r.x();
    v[4] = _r.y();
    v[5] = _r.z();
    v[6] = _r.w();
    return v;
  }

  inline void fromVector(const Vector7d& v) {
    _r = Eigen::Quaterniond(v[6], v[3], v[4], v[5]);
    _t = Vector3D(v[0], v[1], v[2]);
  }

  inline Vector6d toMinimalVector() const {
    Vector6d v;
    v[0] = _t(0);
    v[1] = _t(1);
    v[2] = _t(2);
    v[3] = _r.x();
    v[4] = _r.y();
    v[5] = _r.z();
    return v;
  }

  inline void fromMinimalVector(const Vector6d& v) {
    double w = 1. - v[3] * v[3] - v[4] * v[4] - v[5] * v[5];
    if (w > 0) {
      _r = Eigen::Quaterniond(sqrt(w), v[3], v[4], v[5]);
    } else {
      _r = Eigen::Quaterniond(0, -v[3], -v[4], -v[5]);
    }
    _t = Vector3D(v[0], v[1], v[2]);
  }

  //! Tangent vector (omega, upsilon) whose exponential this is.
  Vector6d log() const {
    const Eigen::AngleAxisd rotation(_r);
    const Vector3D omega = rotation.angle() * rotation.axis();
    Vector6d res;
    res.head<3>() = omega;
    res.tail<3>() = internal::se3V(omega, rotation.angle()).lu().solve(_t);
    return res;
  }

  Vector3D map(const Vector3D& xyz) const { return _r * xyz + _t; }

  //! Exponential of a tangent vector (omega, upsilon).
  static SE3Quat exp(const Vector6d& update) {
    const Vector3D omega = update.head<3>();
    const double theta = omega.norm();
    Eigen::Quaterniond rotation = Eigen::Quaterniond::Identity();
    if (std::fpclassify(theta) != FP_ZERO) {
      rotation = Eigen::AngleAxisd(theta, omega / theta);
    }
    return SE3Quat(rotation, internal::se3V(omega, theta) * update.tail<3>());
  }

  Eigen::Matrix<double, 6, 6, Eigen::ColMajor> adj() const {
    Matrix3D R = _r.toRotationMatrix();
    Eigen::Matrix<double, 6, 6, Eigen::ColMajor> res;
    res.block(0, 0, 3, 3) = R;
    res.block(3, 3, 3, 3) = R;
    res.block(3, 0, 3, 3) = skew(_t) * R;
    res.block(0, 3, 3, 3) = Matrix3D::Zero(3, 3);
    return res;
  }

  Eigen::Matrix<double, 4, 4, Eigen::ColMajor> to_homogeneous_matrix() const {
    Eigen::Matrix<double, 4, 4, Eigen::ColMajor> homogeneous_matrix;
    homogeneous_matrix.setIdentity();
    homogeneous_matrix.block(0, 0, 3, 3) = _r.toRotationMatrix();
    homogeneous_matrix.col(3).head(3) = translation();

    return homogeneous_matrix;
  }

  void normalizeRotation() {
    if (_r.w() < 0) {
      _r.coeffs() *= -1;
    }
    _r.normalize();
  }

  /**
   * cast SE3Quat into an Isometry3D
   */
  operator Isometry3D() const {
    Isometry3D result = (Isometry3D)rotation();
    result.translation() = translation();
    return result;
  }
};

inline std::ostream& operator<<(std::ostream& out_str, const SE3Quat& se3) {
  out_str << se3.to_homogeneous_matrix() << std::endl;
  return out_str;
}

// G2O_TYPES_SLAM3D_API Eigen::Quaterniond euler_to_quat(double yaw, double
// pitch, double roll); G2O_TYPES_SLAM3D_API void quat_to_euler(const
// Eigen::Quaterniond& q, double& yaw, double& pitch, double& roll);
// G2O_TYPES_SLAM3D_API void jac_quat3_euler3(Eigen::Matrix<double, 6, 6,
// Eigen::ColMajor>& J, const SE3Quat& t);

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
