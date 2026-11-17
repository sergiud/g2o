// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TYPES_SLAM3D_ONLINE_H
#define G2O_TYPES_SLAM3D_ONLINE_H

#include <g2o/examples/interactive_slam/g2o_interactive_api.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <iostream>

G2O_START_NAMESPACE

class G2O_INTERACTIVE_API OnlineVertexSE3 : public VertexSE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineVertexSE3()
      : VertexSE3(), updatedEstimate(Eigen::Isometry3d::Identity()) {}

  virtual void oplusImpl(const double* update) {
    VertexSE3::oplusImpl(update);
    updatedEstimate = _estimate;
  }

  void oplusUpdatedEstimate(double* update) {
    Eigen::Map<const Vector6d> v(update);
    Eigen::Isometry3d increment = internal::fromVectorMQT(v);
    updatedEstimate = _estimate * increment;
  }

  VertexSE3::EstimateType updatedEstimate;
};

class G2O_INTERACTIVE_API OnlineEdgeSE3 : public EdgeSE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineEdgeSE3() : EdgeSE3() {}

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* /* to */) {
    OnlineVertexSE3* fromEdge = dynamic_cast<OnlineVertexSE3*>(_vertices[0]);
    OnlineVertexSE3* toEdge = dynamic_cast<OnlineVertexSE3*>(_vertices[1]);
    if (from.count(fromEdge) > 0) {
      toEdge->updatedEstimate = fromEdge->updatedEstimate * _measurement;
      toEdge->setEstimate(toEdge->updatedEstimate);
    } else {
      fromEdge->updatedEstimate = toEdge->updatedEstimate * _inverseMeasurement;
      fromEdge->setEstimate(fromEdge->updatedEstimate);
    }
  }

  double chi2() const {
    OnlineVertexSE3* from = dynamic_cast<OnlineVertexSE3*>(_vertices[0]);
    OnlineVertexSE3* to = dynamic_cast<OnlineVertexSE3*>(_vertices[1]);
    Eigen::Isometry3d delta =
        _inverseMeasurement * from->estimate().inverse() * to->estimate();
    Vector6d error = internal::toVectorMQT(delta);
    return error.dot(information() * error);
  }
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
