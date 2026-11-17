// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_LINE3D_H_
#define G2O_VERTEX_LINE3D_H_

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>
#include <g2o/types/slam3d_addons/line3d.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM3D_ADDONS_API VertexLine3D : public BaseVertex<4, Line3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  VertexLine3D();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setToOriginImpl() { _estimate = Line3D(); }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<const Eigen::Vector4d> update(update_);
    _estimate.oplus(update);
  }

  virtual bool setEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector6d> _est(est);
    _estimate = Line3D(_est);
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector6d> _est(est);
    _est = _estimate;
    return true;
  }

  virtual int estimateDimension() const { return 6; }

  Eigen::Vector3d color;
};

#ifdef G2O_HAVE_OPENGL
class VertexLine3DDrawAction : public DrawAction {
 public:
  VertexLine3DDrawAction();
  virtual HyperGraphElementAction* operator()(
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
