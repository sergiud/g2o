// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_LINE2D_H
#define G2O_VERTEX_LINE2D_H

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/stuff/misc.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h>

#include <Eigen/Core>

G2O_START_NAMESPACE

class G2O_TYPES_SLAM2D_ADDONS_API VertexLine2D : public BaseVertex<2, Line2D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexLine2D();

  double theta() const { return _estimate[0]; }
  void setTheta(double t) { _estimate[0] = t; }

  double rho() const { return _estimate[1]; }
  void setRho(double r) { _estimate[1] = r; }

  virtual void setToOriginImpl() { _estimate.setZero(); }

  virtual bool setEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector2D> v(est);
    _estimate = Line2D(v);
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector2D> v(est);
    v = _estimate;
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
    _estimate += Eigen::Map<const Vector2D>(update);
    _estimate(0) = normalize_theta(_estimate(0));
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
  int p1Id, p2Id;
};

/* class G2O_TYPES_SLAM2D_ADDONS_API VertexLine2DWriteGnuplotAction: public
 * WriteGnuplotAction { */
/* public: */
/*   VertexLine2DWriteGnuplotAction(); */
/*   virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement*
 * element,  */
/*           HyperGraphElementAction::Parameters* params_); */
/* }; */

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM2D_ADDONS_API VertexLine2DDrawAction : public DrawAction {
 public:
  VertexLine2DDrawAction();
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
