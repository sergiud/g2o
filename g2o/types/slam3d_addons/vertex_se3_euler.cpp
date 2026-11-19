// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o/types/slam3d_addons/vertex_se3_euler.h>

#include <iostream>

using namespace Eigen;

G2O_START_NAMESPACE

bool VertexSE3Euler::read(std::istream& is) {
  Vector6d est;
  for (int i = 0; i < 6; i++) is >> est[i];
  Isometry3D transf = g2o::internal::fromVectorET(est);
  setEstimate(transf);
  updateCache();
  return !is.fail();
}

bool VertexSE3Euler::write(std::ostream& os) const {
  Vector6d est = g2o::internal::toVectorET(estimate());
  for (int i = 0; i < 6; i++) os << est[i] << " ";
  return os.good();
}

G2O_END_NAMESPACE
