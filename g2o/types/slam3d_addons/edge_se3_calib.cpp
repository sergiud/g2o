// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d_addons/edge_se3_calib.h>

G2O_START_NAMESPACE
#ifdef _MSC_VER
extern template const BaseEdge<6, Isometry3D>::InformationType&
BaseEdge<6, Isometry3D>::information() const;
#endif  // defined(_MSC_VER)
EdgeSE3Calib::EdgeSE3Calib() : BaseMultiEdge<6, Isometry3D>() { resize(3); }

void EdgeSE3Calib::computeError() {
  const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
  const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
  const VertexSE3* calib = dynamic_cast<const VertexSE3*>(_vertices[2]);
  _error = g2o::internal::toVectorMQT(
      _measurement.inverse() * calib->estimate().inverse() *
      v1->estimate().inverse() * v2->estimate() * calib->estimate());
}

bool EdgeSE3Calib::write(std::ostream& os) const {
  Vector7d meas = g2o::internal::toVectorQT(_measurement);
  for (int i = 0; i < 7; i++) os << meas[i] << " ";
  for (int i = 0; i < information().rows(); i++)
    for (int j = i; j < information().cols(); j++) {
      os << information()(i, j) << " ";
    }
  return os.good();
}

bool EdgeSE3Calib::read(std::istream& is) {
  Vector7d meas;
  for (int i = 0; i < 7; i++) is >> meas[i];
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4D::MapType(meas.data() + 3).normalize();
  setMeasurement(g2o::internal::fromVectorQT(meas));

  if (is.bad()) {
    return false;
  }
  for (int i = 0; i < information().rows() && is.good(); i++)
    for (int j = i; j < information().cols() && is.good(); j++) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  if (is.bad()) {
    //  we overwrite the information matrix with the Identity
    information().setIdentity();
  }
  return true;
}

G2O_END_NAMESPACE
