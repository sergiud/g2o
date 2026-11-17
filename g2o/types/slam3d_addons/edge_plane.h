// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_PLANE3D_H
#define G2O_EDGE_PLANE3D_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM3D_ADDONS_API EdgePlane
    : public BaseBinaryEdge<4, Vector4D, VertexPlane, VertexPlane> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePlane();

  void computeError() {
    const VertexPlane* v1 = dynamic_cast<const VertexPlane*>(_vertices[0]);
    const VertexPlane* v2 = dynamic_cast<const VertexPlane*>(_vertices[1]);
    _error =
        (v2->estimate().toVector() - v1->estimate().toVector()) - _measurement;
  }
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setMeasurement(const Vector4D& m) { _measurement = m; }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector4D> m(d);
    _measurement = m;
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector4D> m(d);
    m = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 4; }

  virtual bool setMeasurementFromState() {
    const VertexPlane* v1 = dynamic_cast<const VertexPlane*>(_vertices[0]);
    const VertexPlane* v2 = dynamic_cast<const VertexPlane*>(_vertices[1]);
    _measurement = (v2->estimate().toVector()) - v1->estimate().toVector();

    return true;
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 0.;
  }
#if 0
#ifndef NUMERIC_JACOBIAN_THREE_D_TYPES
    virtual void linearizeOplus();
#endif
#endif
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
