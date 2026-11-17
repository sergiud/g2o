// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_PRIOR_H_
#define G2O_EDGE_SE3_PRIOR_H_

#include <g2o/core/base_unary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_se3.h>

G2O_START_NAMESPACE
/**
 * \brief prior for an SE3 element
 *
 * Provides a prior for a 3d pose vertex. Again the measurement is represented
 * by an Isometry3D matrix.
 */
class G2O_TYPES_SLAM3D_API EdgeSE3Prior
    : public BaseUnaryEdge<6, Isometry3D, VertexSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3Prior();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  // return the error estimate as a 3-vector
  void computeError();

  // jacobian
  virtual void linearizeOplus();

  virtual void setMeasurement(const Isometry3D& m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector7d> v(d);
    _measurement = internal::fromVectorQT(v);
    _inverseMeasurement = _measurement.inverse();
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector7d> v(d);
    v = internal::toVectorQT(_measurement);
    return true;
  }

  virtual int measurementDimension() const { return 7; }

  virtual bool setMeasurementFromState();

  virtual double initialEstimatePossible(
      const OptimizableGraph::VertexSet& /*from*/,
      OptimizableGraph::Vertex* /*to*/) {
    return 1.;
  }

  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* to);

 protected:
  Isometry3D _inverseMeasurement;
  virtual bool resolveCaches();
  ParameterSE3Offset* _offsetParam;
  CacheSE3Offset* _cache;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
