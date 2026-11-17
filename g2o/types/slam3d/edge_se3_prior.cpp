// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/isometry3d_gradients.h>

#include <iostream>

G2O_START_NAMESPACE
using namespace std;

// point to camera projection, monocular
EdgeSE3Prior::EdgeSE3Prior() : BaseUnaryEdge<6, Isometry3D, VertexSE3>() {
  setMeasurement(Isometry3D::Identity());
  information().setIdentity();
  _cache = nullptr;
  _offsetParam = nullptr;
  resizeParameters(1);
  installParameter(_offsetParam, 0);
}

bool EdgeSE3Prior::resolveCaches() {
  assert(_offsetParam);
  ParameterVector pv(1);
  pv[0] = _offsetParam;
  resolveCache(_cache, dynamic_cast<OptimizableGraph::Vertex*>(_vertices[0]),
               "CACHE_SE3_OFFSET", pv);
  return _cache != nullptr;
}

bool EdgeSE3Prior::read(std::istream& is) {
  int pid;
  is >> pid;
  if (!setParameterId(0, pid)) return false;
  // measured keypoint
  Vector7d meas;
  for (int i = 0; i < 7; i++) is >> meas[i];
  setMeasurement(internal::fromVectorQT(meas));
  // don't need this if we don't use it in error calculation (???)
  // information matrix is the identity for features, could be changed to allow
  // arbitrary covariances
  if (is.bad()) {
    return false;
  }
  for (int i = 0; i < information().rows() && is.good(); i++)
    for (int j = i; j < information().cols() && is.good(); j++) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  if (is.bad()) {
    //  we overwrite the information matrix
    information().setIdentity();
  }
  return true;
}

bool EdgeSE3Prior::write(std::ostream& os) const {
  os << _offsetParam->id() << " ";
  Vector7d meas = internal::toVectorQT(_measurement);
  for (int i = 0; i < 7; i++) os << meas[i] << " ";
  for (int i = 0; i < information().rows(); i++)
    for (int j = i; j < information().cols(); j++) {
      os << information()(i, j) << " ";
    }
  return os.good();
}

void EdgeSE3Prior::computeError() {
  Isometry3D delta = _inverseMeasurement * _cache->n2w();
  _error = internal::toVectorMQT(delta);
}

void EdgeSE3Prior::linearizeOplus() {
  VertexSE3* from = dynamic_cast<VertexSE3*>(_vertices[0]);
  Isometry3D E;
  Isometry3D Z, X, P;
  X = from->estimate();
  P = _cache->offsetParam()->offset();
  Z = _measurement;
  internal::computeEdgeSE3PriorGradient(E, _jacobianOplusXi, Z, X, P);
}

bool EdgeSE3Prior::setMeasurementFromState() {
  setMeasurement(_cache->n2w());
  return true;
}

void EdgeSE3Prior::initialEstimate(const OptimizableGraph::VertexSet& /*from_*/,
                                   OptimizableGraph::Vertex* /*to_*/) {
  VertexSE3* v = dynamic_cast<VertexSE3*>(_vertices[0]);
  assert(v && "Vertex for the Prior edge is not set");

  Isometry3D newEstimate = _offsetParam->offset().inverse() * measurement();
  if (_information.block<3, 3>(0, 0).array().abs().sum() ==
      0) {  // do not set translation, as that part of the information is all
            // zero
    newEstimate.translation() = v->estimate().translation();
  }
  if (_information.block<3, 3>(3, 3).array().abs().sum() ==
      0) {  // do not set rotation, as that part of the information is all zero
    newEstimate.matrix().block<3, 3>(0, 0) =
        internal::extractRotation(v->estimate());
  }
  v->setEstimate(newEstimate);
}

G2O_END_NAMESPACE
