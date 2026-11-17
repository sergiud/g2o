// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_SE2_H
#define G2O_VERTEX_SE2_H

#include <g2o/config.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/se2.h>

G2O_START_NAMESPACE

/**
 * \brief 2D pose Vertex, (x,y,theta)
 */
class G2O_TYPES_SLAM2D_API VertexSE2 : public BaseVertex<3, SE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VertexSE2();

  virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double* update) {
    Vector2D t = _estimate.translation();
    t += Eigen::Map<const Vector2D>(update);
    double angle = normalize_theta(_estimate.rotation().angle() + update[2]);
    _estimate.setTranslation(t);
    _estimate.setRotation(Eigen::Rotation2Dd(angle));
  }

  virtual bool setEstimateDataImpl(const double* est) {
    _estimate = SE2(est[0], est[1], est[2]);
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector3D> v(est);
    v = _estimate.toVector();
    return true;
  }

  virtual int estimateDimension() const { return 3; }

  virtual bool setMinimalEstimateDataImpl(const double* est) {
    return setEstimateData(est);
  }

  virtual bool getMinimalEstimateData(double* est) const {
    return getEstimateData(est);
  }

  virtual int minimalEstimateDimension() const { return 3; }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

class G2O_TYPES_SLAM2D_API VertexSE2WriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexSE2WriteGnuplotAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_API VertexSE2DrawAction : public DrawAction {
 public:
  VertexSE2DrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  HyperGraphElementAction* _drawActions;
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty *_triangleX{}, *_triangleY{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
