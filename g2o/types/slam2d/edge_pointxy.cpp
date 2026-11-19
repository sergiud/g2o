// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2025 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d/edge_pointxy.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

G2O_START_NAMESPACE

EdgePointXY::EdgePointXY()
    : BaseBinaryEdge<2, Vector2D, VertexPointXY, VertexPointXY>() {
  _information.setIdentity();
  _error.setZero();
}

bool EdgePointXY::read(std::istream& is) {
  Vector2D p;
  is >> p[0] >> p[1];
  setMeasurement(p);
  for (int i = 0; i < 2; ++i)
    for (int j = i; j < 2; ++j) {
      is >> information()(i, j);
      if (!is) return false;
      if (i != j) information()(j, i) = information()(i, j);
    }
  return !is.fail();
}

bool EdgePointXY::write(std::ostream& os) const {
  Vector2D p = measurement();
  os << p.x() << " " << p.y();
  for (int i = 0; i < 2; ++i)
    for (int j = i; j < 2; ++j) os << " " << information()(i, j);
  return os.good();
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgePointXY::linearizeOplus() {
  _jacobianOplusXi = -Matrix2D::Identity();
  _jacobianOplusXj = Matrix2D::Identity();
}
#endif

G2O_END_NAMESPACE
