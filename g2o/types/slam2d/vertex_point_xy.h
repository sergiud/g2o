// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_POINT_XY_H
#define G2O_VERTEX_POINT_XY_H

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/g2o_types_slam2d_api.h>
#include <g2o/types/slam2d_addons/line_2d.h>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM2D_API VertexPointXY : public BaseVertex<2, Vector2D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexPointXY();

  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual bool setEstimateDataImpl(const double* est) {
    _estimate[0] = est[0];
    _estimate[1] = est[1];
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    est[0] = _estimate[0];
    est[1] = _estimate[1];
    return true;
  }

  virtual int estimateDimension() const { return 2; }

  virtual bool setMinimalEstimateDataImpl(const double* est) {
    return setEstimateData(est);
  }

  virtual bool getMinimalEstimateData(double* est) const {
    return getEstimateData(est);
  }

  virtual int minimalEstimateDimension() const { return 2; }

  virtual void oplusImpl(const double* update) {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

class G2O_TYPES_SLAM2D_API VertexPointXYWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexPointXYWriteGnuplotAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API VertexPointXYDrawAction : public DrawAction {
 public:
  VertexPointXYDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  FloatProperty* _pointSize{};
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
