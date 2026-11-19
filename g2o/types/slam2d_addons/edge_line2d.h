// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_LINE2D_H
#define G2O_EDGE_LINE2D_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>
#include <g2o/stuff/misc.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>
#include <g2o/types/slam2d_addons/vertex_line2d.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2D
    : public BaseBinaryEdge<2, Line2D, VertexLine2D, VertexLine2D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeLine2D();

  void computeError() {
    const VertexLine2D* v1 = dynamic_cast<const VertexLine2D*>(_vertices[0]);
    const VertexLine2D* v2 = dynamic_cast<const VertexLine2D*>(_vertices[1]);
    _error = (v2->estimate() - v1->estimate()) - _measurement;
    _error[0] = normalize_theta(_error[0]);
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setMeasurement(const Line2D& m) { _measurement = m; }

  virtual void setMeasurement(const Vector2D& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector2D> m(d);
    _measurement = Line2D(m);
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector2D> m(d);
    m = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 2; }

  virtual bool setMeasurementFromState() {
    const VertexLine2D* v1 = dynamic_cast<const VertexLine2D*>(_vertices[0]);
    const VertexLine2D* v2 = dynamic_cast<const VertexLine2D*>(_vertices[1]);
    _measurement =
        Line2D((Vector2D)(v2->estimate()) - (Vector2D)v1->estimate());
    return true;
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 0.;
  }
#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
  virtual void linearizeOplus();
#endif
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
