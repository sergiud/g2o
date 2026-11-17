// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE2_PRIOR_XY_H
#define G2O_EDGE_SE2_PRIOR_XY_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/g2o_types_slam2d_api.h>
#include <g2o/types/slam2d/vertex_se2.h>

G2O_START_NAMESPACE

/**
 * \brief Prior for a two D pose with constraints only in xy direction (like
 * gps)
 */
class G2O_TYPES_SLAM2D_API EdgeSE2XYPrior
    : public BaseUnaryEdge<2, Vector2D, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2XYPrior();

  virtual bool setMeasurementData(const double* d) {
    _measurement[0] = d[0];
    _measurement[1] = d[1];
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    d[0] = _measurement[0];
    d[1] = _measurement[1];
    return true;
  }

  virtual int measurementDimension() const { return 2; }

  virtual void linearizeOplus();

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void computeError() {
    const VertexSE2* v = dynamic_cast<const VertexSE2*>(_vertices[0]);
    _error = v->estimate().translation() - _measurement;
  }
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
