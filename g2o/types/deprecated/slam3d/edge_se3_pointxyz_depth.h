// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DEPRECATED_EDGE_PROJECT_DEPTH_H_
#define G2O_DEPRECATED_EDGE_PROJECT_DEPTH_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>

#include "parameter_camera.h"
#include "vertex_pointxyz.h"
#include "vertex_se3_quat.h"

G2O_START_NAMESPACE
namespace deprecated {

/*! \class EdgeProjectDepth
 * \brief g2o edge from a track to a depth camera node using a depth measurement
 * (true distance, not disparity)
 */
// first two args are the measurement type, second two the connection classes
class G2O_DEPRECATED_TYPES_SLAM3D_API EdgeSE3PointXYZDepth
    : public BaseBinaryEdge<3, Eigen::Vector3d, VertexSE3, VertexPointXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3PointXYZDepth();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  // return the error estimate as a 3-vector
  void computeError();
  // jacobian
  virtual void linearizeOplus();

  virtual void setMeasurement(const Eigen::Vector3d& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Eigen::Vector3d> v(d);
    _measurement = v;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Eigen::Vector3d> v(d);
    v = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 3; }

  virtual bool setMeasurementFromState();

  virtual double initialEstimatePossible(
      const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
    (void)to;
    return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
  }

  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* to);

 private:
  Eigen::Matrix<double, 3, 9> J;  // jacobian before projection

  virtual bool resolveCaches();
  ParameterCamera* params;
  CacheCamera* cache;
};

}  // namespace deprecated
G2O_END_NAMESPACE
#endif
