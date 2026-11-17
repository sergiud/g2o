// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_PLANE_CALIB_H
#define G2O_EDGE_SE3_PLANE_CALIB_H

#include <g2o/config.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

G2O_START_NAMESPACE
/**
 * \brief plane measurement that also calibrates an offset for the sensor
 */
class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalib
    : public BaseMultiEdge<3, Plane3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE3PlaneSensorCalib();
  Vector3D color;

  void computeError() {
    const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
    const VertexPlane* planeVertex =
        dynamic_cast<const VertexPlane*>(_vertices[1]);
    const VertexSE3* offset = dynamic_cast<const VertexSE3*>(_vertices[2]);
    const Plane3D& plane = planeVertex->estimate();
    // measurement function: remap the plane in global coordinates
    Isometry3D w2n = (v1->estimate() * offset->estimate()).inverse();
    Plane3D localPlane = w2n * plane;
    _error = localPlane.ominus(_measurement);
  }

  void setMeasurement(const Plane3D& m) { _measurement = m; }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

#ifdef G2O_HAVE_OPENGL
class EdgeSE3PlaneSensorCalibDrawAction : public DrawAction {
 public:
  G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3PlaneSensorCalibDrawAction();
  G2O_TYPES_SLAM3D_ADDONS_API virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty *_planeWidth{}, *_planeHeight{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
