// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2025 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d_addons/edge_line2d.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

using namespace g2o;
using namespace Eigen;

EdgeLine2D::EdgeLine2D()
    : BaseBinaryEdge<2, Line2D, VertexLine2D, VertexLine2D>() {
  _information.setIdentity();
  _error.setZero();
}

bool EdgeLine2D::read(std::istream& is) {
  Vector2D v;
  for (int i = 0; i < 2; ++i) is >> v[i];
  setMeasurement(v);
  for (int i = 0; i < 2; ++i)
    for (int j = i; j < 2; ++j) {
      is >> information()(i, j);
      if (!is) return false;
      if (i != j) information()(j, i) = information()(i, j);
    }
  return !is.fail();
}

bool EdgeLine2D::write(std::ostream& os) const {
  for (int i = 0; i < 2; ++i) os << _measurement[i] << " ";
  for (int i = 0; i < 2; ++i)
    for (int j = i; j < 2; ++j) os << " " << information()(i, j);
  return os.good();
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeLine2D::linearizeOplus() {
  _jacobianOplusXi = -Matrix2d::Identity();
  _jacobianOplusXj = Matrix2d::Identity();
}
#endif
