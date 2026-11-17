// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_POINTXYZ_H
#define G2O_EDGE_POINTXYZ_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM3D_API EdgePointXYZ
    : public BaseBinaryEdge<3, Vector3D, VertexPointXYZ, VertexPointXYZ> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointXYZ();

  void computeError() {
    const VertexPointXYZ* v1 =
        dynamic_cast<const VertexPointXYZ*>(_vertices[0]);
    const VertexPointXYZ* v2 =
        dynamic_cast<const VertexPointXYZ*>(_vertices[1]);
    _error = (v2->estimate() - v1->estimate()) - _measurement;
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setMeasurement(const Vector3D& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    _measurement = Vector3D(d[0], d[1], d[2]);
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector3D> m(d);
    m = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 3; }

  virtual bool setMeasurementFromState() {
    const VertexPointXYZ* v1 =
        dynamic_cast<const VertexPointXYZ*>(_vertices[0]);
    const VertexPointXYZ* v2 =
        dynamic_cast<const VertexPointXYZ*>(_vertices[1]);
    _measurement = v2->estimate() - v1->estimate();
    return true;
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 0.;
  }
#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
  virtual void linearizeOplus();
#endif
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
