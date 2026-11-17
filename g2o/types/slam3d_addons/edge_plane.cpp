// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d_addons/edge_plane.h>

G2O_START_NAMESPACE
using namespace Eigen;

EdgePlane::EdgePlane() :
    BaseBinaryEdge<4, Vector4D, VertexPlane, VertexPlane>()
{
    _information.setIdentity();
    _error.setZero();
}

bool EdgePlane::read(std::istream& is)
{
  Vector4D  v;
    int size=4;
    for (int i = 0; i < size; ++i)
        is >> v[i];
    setMeasurement(v);
    for (int i = 0; i < size; ++i)
        for (int j = i; j < size; ++j) {
            is >> information()(i, j);
            if (i != j)
                information()(j, i) = information()(i, j);
        }
    return true;
}

bool EdgePlane::write(std::ostream& os) const
{
    int size=4;
    for (int i = 0; i < size; ++i)
        os << _measurement[i] << " ";
    for (int i = 0; i < size; ++i)
        for (int j = i; j < size; ++j)
            os << " " << information()(i, j);
    return os.good();
}

#if 0
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgePlane::linearizeOplus()
{
    _jacobianOplusXi=-Matrix3D::Identity();
    _jacobianOplusXj= Matrix3D::Identity();
}
#endif
#endif


G2O_END_NAMESPACE
