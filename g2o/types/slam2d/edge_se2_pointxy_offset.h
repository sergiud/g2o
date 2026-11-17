// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE2_POINT_XY_OFFSET_H_
#define G2O_EDGE_SE2_POINT_XY_OFFSET_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/g2o_types_slam2d_api.h>
#include <g2o/types/slam2d/parameter_se2_offset.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

#include <Eigen/Geometry>

G2O_START_NAMESPACE

/*! \class EdgeSE2TrackXYZ
 * \brief g2o edge from a track to a point node
 */
// first two args are the measurement type, second two the connection classes
class G2O_TYPES_SLAM2D_API EdgeSE2PointXYOffset
    : public BaseBinaryEdge<2, Vector2D, VertexSE2, VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE2PointXYOffset();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  // return the error estimate as a 3-vector
  void computeError();
  // jacobian
  virtual void linearizeOplus();

  virtual void setMeasurement(const Vector2D& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector2D> v(d);
    _measurement = v;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector2D> v(d);
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
  ParameterSE2Offset* offsetParam;
  CacheSE2Offset* cache;
  virtual bool resolveCaches();
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
