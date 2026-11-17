// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2015-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/eigen_types.h>
#include <g2o/types/slam3d_addons/line3d.h>

#include <iostream>

G2O_START_NAMESPACE

using namespace std;
using namespace Eigen;

static inline void _skew(Matrix3d& S, const Vector3d& t) {
  S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
}

static inline Matrix3d _skew(const Vector3d& t) {
  Matrix3d S;
  S << 0, -t.z(), t.y(), t.z(), 0, -t.x(), -t.y(), t.x(), 0;
  return S;
}

Vector6d Line3D::toCartesian() const {
  Vector6d cartesian;
  cartesian.tail<3>() = d() / d().norm();
  Eigen::Matrix3d W = -_skew(d());
  double damping = 1e-9;
  Eigen::Matrix3d A =
      W.transpose() * W + (Eigen::Matrix3d::Identity() * damping);
  cartesian.head<3>() = A.ldlt().solve(W.transpose() * w());
  return cartesian;
}

Line3D operator*(const Eigen::Isometry3d& t, const Line3D& line) {
  Matrix6d A = Matrix6d::Zero();
  A.block<3, 3>(0, 0) = t.linear();
  A.block<3, 3>(0, 3) = _skew(t.translation()) * t.linear();
  A.block<3, 3>(3, 3) = t.linear();
  Vector6d v = (Vector6d)line;
  return Line3D(A * v);
}

namespace internal {
Vector6d transformCartesianLine(const Eigen::Isometry3d& t,
                                const Vector6d& line) {
  Vector6d l;
  l.head<3>() = t * line.head<3>();
  l.tail<3>() = t.linear() * line.tail<3>();
  return normalizeCartesianLine(l);
}

Vector6d normalizeCartesianLine(const Vector6d& line) {
  Vector3d p0 = line.head<3>();
  Vector3d d0 = line.tail<3>();
  d0.normalize();
  p0 -= d0 * (d0.dot(p0));
  Vector6d nl;
  nl.head<3>() = p0;
  nl.tail<3>() = d0;
  return nl;
}

}  // namespace internal

G2O_END_NAMESPACE
