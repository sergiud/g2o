// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TYPES_SLAM2D_ONLINE_H
#define G2O_TYPES_SLAM2D_ONLINE_H

#include <g2o/examples/interactive_slam/g2o_interactive_api.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/edge_se2.h>

#include <iostream>

G2O_START_NAMESPACE

class G2O_INTERACTIVE_API OnlineVertexSE2 : public VertexSE2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineVertexSE2() : VertexSE2() {}

  virtual void oplusImpl(const double* update) {
    VertexSE2::oplusImpl(update);
    updatedEstimate = _estimate;
  }

  void oplusUpdatedEstimate(double* update) {
    Eigen::Vector3d p = _estimate.toVector();
    p += Eigen::Map<Eigen::Vector3d>(update);
    p[2] = normalize_theta(p[2]);
    updatedEstimate.fromVector(p);
    // std::cerr << PVAR(updatedEstimate.toVector()) << " " <<
    // PVAR(_estimate.toVector()) << std::endl;
  }

  VertexSE2::EstimateType updatedEstimate;
};

class G2O_INTERACTIVE_API OnlineEdgeSE2 : public EdgeSE2 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  OnlineEdgeSE2() : EdgeSE2() {}

  void initialEstimate(const OptimizableGraph::VertexSet& from,
                       OptimizableGraph::Vertex* /* to */) {
    OnlineVertexSE2* fromEdge = dynamic_cast<OnlineVertexSE2*>(_vertices[0]);
    OnlineVertexSE2* toEdge = dynamic_cast<OnlineVertexSE2*>(_vertices[1]);
    if (from.count(fromEdge) > 0) {
      toEdge->updatedEstimate = fromEdge->updatedEstimate * _measurement;
      toEdge->setEstimate(toEdge->updatedEstimate);
    } else {
      fromEdge->updatedEstimate = toEdge->updatedEstimate * _inverseMeasurement;
      fromEdge->setEstimate(fromEdge->updatedEstimate);
    }
  }

  double chi2() const {
    const OnlineVertexSE2* v1 =
        dynamic_cast<const OnlineVertexSE2*>(_vertices[0]);
    const OnlineVertexSE2* v2 =
        dynamic_cast<const OnlineVertexSE2*>(_vertices[1]);
    SE2 delta = _inverseMeasurement *
                (v1->updatedEstimate.inverse() * v2->updatedEstimate);
    Eigen::Vector3d error = delta.toVector();
    return error.dot(information() * error);
  }
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
