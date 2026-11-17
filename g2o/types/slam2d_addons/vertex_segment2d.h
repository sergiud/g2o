// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_SEGMENT_2D_H
#define G2O_VERTEX_SEGMENT_2D_H

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h>

#include <Eigen/Core>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2D
    : public BaseVertex<4, Vector4D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexSegment2D();

  Vector2D estimateP1() const {
    return Eigen::Map<const Vector2D>(&(_estimate[0]));
  }
  Vector2D estimateP2() const {
    return Eigen::Map<const Vector2D>(&(_estimate[2]));
  }
  void setEstimateP1(const Vector2D& p1) {
    Eigen::Map<Vector2D> v(&_estimate[0]);
    v = p1;
  }
  void setEstimateP2(const Vector2D& p2) {
    Eigen::Map<Vector2D> v(&_estimate[2]);
    v = p2;
  }

  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual bool setEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector4D> v(est);
    _estimate = v;
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector4D> v(est);
    v = _estimate;
    return true;
  }

  virtual int estimateDimension() const { return 4; }

  virtual bool setMinimalEstimateDataImpl(const double* est) {
    return setEstimateData(est);
  }

  virtual bool getMinimalEstimateData(double* est) const {
    return getEstimateData(est);
  }

  virtual int minimalEstimateDimension() const { return 4; }

  virtual void oplusImpl(const double* update) {
    Eigen::Map<const Vector4D> upd(update);
    _estimate += upd;
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2DWriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexSegment2DWriteGnuplotAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_ADDONS_API VertexSegment2DDrawAction
    : public DrawAction {
 public:
  VertexSegment2DDrawAction();
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
