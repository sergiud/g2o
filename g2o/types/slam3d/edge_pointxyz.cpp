// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_pointxyz.h>

G2O_START_NAMESPACE

EdgePointXYZ::EdgePointXYZ()
    : BaseBinaryEdge<3, Vector3D, VertexPointXYZ, VertexPointXYZ>() {
  _information.setIdentity();
  _error.setZero();
}

bool EdgePointXYZ::read(std::istream& is) {
  Vector3D p;
  is >> p[0] >> p[1] >> p[2];
  setMeasurement(p);
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return !is.fail();
}

bool EdgePointXYZ::write(std::ostream& os) const {
  Vector3D p = measurement();
  os << p.x() << " " << p.y() << " " << p.z();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) os << " " << information()(i, j);
  return os.good();
}

#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
void EdgePointXYZ::linearizeOplus() {
  _jacobianOplusXi = -Matrix3D::Identity();
  _jacobianOplusXj = Matrix3D::Identity();
}
#endif

G2O_END_NAMESPACE
