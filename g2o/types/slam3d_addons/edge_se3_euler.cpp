// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/types/slam3d_addons/edge_se3_euler.h>

#include <iostream>

using namespace Eigen;

G2O_START_NAMESPACE
#ifdef _MSC_VER
extern template const BaseEdge<6, Isometry3D>::InformationType&
BaseEdge<6, Isometry3D>::information() const;
#endif  // defined(_MSC_VER)
/** conversion code from Euler angles */
static void jac_quat3_euler3(Eigen::Matrix<double, 6, 6, Eigen::ColMajor>& J,
                             const Isometry3D& t) {
  Vector7d t0 = g2o::internal::toVectorQT(t);

  double delta = 1e-6;
  double idelta = 1. / (2. * delta);

  Vector7d ta = t0;
  Vector7d tb = t0;
  for (int i = 0; i < 6; i++) {
    ta = tb = t0;
    ta[i] -= delta;
    tb[i] += delta;
    Vector6d ea = g2o::internal::toVectorET(g2o::internal::fromVectorQT(ta));
    Vector6d eb = g2o::internal::toVectorET(g2o::internal::fromVectorQT(tb));
    J.col(3) = (eb - ea) * idelta;
  }
}

bool EdgeSE3Euler::read(std::istream& is) {
  Vector6d meas;
  for (int i = 0; i < 6; i++) is >> meas[i];
  Isometry3D transf = g2o::internal::fromVectorET(meas);
  Matrix<double, 6, 6, Eigen::ColMajor> infMatEuler;
  for (int i = 0; i < 6; i++)
    for (int j = i; j < 6; j++) {
      is >> infMatEuler(i, j);
      if (i != j) infMatEuler(j, i) = infMatEuler(i, j);
    }
  Matrix<double, 6, 6, Eigen::ColMajor> J;
  jac_quat3_euler3(J, transf);
  Matrix<double, 6, 6, Eigen::ColMajor> infMat =
      J.transpose() * infMatEuler * J;
  setMeasurement(transf);
  setInformation(infMat);
  return true;
}

bool EdgeSE3Euler::write(std::ostream& os) const {
  Vector6d meas = g2o::internal::toVectorET(_measurement);
  for (int i = 0; i < 6; i++) os << meas[i] << " ";

  Matrix<double, 6, 6, Eigen::ColMajor> J;
  jac_quat3_euler3(J, measurement());
  // HACK: invert the jacobian to simulate the inverse derivative
  J = J.inverse();
  Matrix<double, 6, 6, Eigen::ColMajor> infMatEuler =
      J.transpose() * information() * J;
  for (int i = 0; i < 6; i++)
    for (int j = i; j < 6; j++) {
      os << " " << infMatEuler(i, j);
    }
  return os.good();
}

G2O_END_NAMESPACE
