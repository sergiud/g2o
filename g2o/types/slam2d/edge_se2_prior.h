// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE2_PRIOR_H
#define G2O_EDGE_SE2_PRIOR_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/g2o_types_slam2d_api.h>
#include <g2o/types/slam2d/vertex_se2.h>

G2O_START_NAMESPACE

/**
 * \brief Prior for a two D pose
 */
class G2O_TYPES_SLAM2D_API EdgeSE2Prior
    : public BaseUnaryEdge<3, SE2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2Prior();

  void computeError() {
    const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
    SE2 delta = _inverseMeasurement * v1->estimate();
    _error = delta.toVector();
  }

  virtual void linearizeOplus() {
    _jacobianOplusXi.setZero();
    _jacobianOplusXi.block<2, 2>(0, 0) =
        _inverseMeasurement.rotation().toRotationMatrix();
    _jacobianOplusXi(2, 2) = 1.;
  }

  virtual void setMeasurement(const SE2& m);
  virtual bool setMeasurementData(const double* d);

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector3D> v(d);
    v = _measurement.toVector();
    return true;
  }

  int measurementDimension() const { return 3; }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 1.;
  }
  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* to);

 protected:
  SE2 _inverseMeasurement;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
