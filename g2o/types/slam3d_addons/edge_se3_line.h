// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_LINE_H_
#define G2O_EDGE_SE3_LINE_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>
#include <g2o/types/slam3d_addons/line3d.h>
#include <g2o/types/slam3d_addons/vertex_line3d.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Line3D
    : public BaseBinaryEdge<4, Line3D, VertexSE3, VertexLine3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  EdgeSE3Line3D();

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  void computeError();

  virtual void setMeasurement(const Vector6d& m) { _measurement = m; }

  virtual void setMeasurement(const Line3D& m) { _measurement = Line3D(m); }

  virtual bool setMeasurementData(const double* d) {
    Eigen::Map<const Vector6d> v(d);
    _measurement = Line3D(v);
    return true;
  }

  virtual bool getMeasurementData(double* d) const {
    Eigen::Map<Vector6d> v(d);
    v = _measurement;
    return true;
  }

  virtual int measurementDimension() const { return 6; }

  Eigen::Vector3d color;

 private:
  ParameterSE3Offset* offsetParam;
  CacheSE3Offset* cache;
  virtual bool resolveCaches();
};

#ifdef G2O_HAVE_OPENGL
class EdgeSE3Line3DDrawAction : public DrawAction {
 public:
  G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Line3DDrawAction();
  G2O_TYPES_SLAM3D_ADDONS_API virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty *_lineLength{}, *_lineWidth{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
