// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DEPRECATED_VERTEX_SE3_QUAT_
#define G2O_DEPRECATED_VERTEX_SE3_QUAT_

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/deprecated/g2o_deprecated_types_slam3d_api.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/se3quat.h>

G2O_START_NAMESPACE
namespace deprecated {

/**
 * \brief 3D pose Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 6d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 */
class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3
    : public BaseVertex<6, SE3Quat> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexSE3();

  virtual void setToOriginImpl() { _estimate = SE3Quat(); }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual bool setEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector7d> v(est);
    _estimate.fromVector(v);
    return true;
  }

  virtual bool getEstimateData(double* est) const {
    Eigen::Map<Vector7d> v(est);
    v = _estimate.toVector();
    return true;
  }

  virtual int estimateDimension() const { return 7; }

  virtual bool setMinimalEstimateDataImpl(const double* est) {
    Eigen::Map<const Vector6d> v(est);
    _estimate.fromMinimalVector(v);
    return true;
  }

  virtual bool getMinimalEstimateData(double* est) const {
    Eigen::Map<Vector6d> v(est);
    v = _estimate.toMinimalVector();
    return true;
  }

  virtual int minimalEstimateDimension() const { return 6; }

  virtual void oplusImpl(const double* update) {
    Eigen::Map<const Vector6d> v(update);
    SE3Quat increment(v);
    _estimate *= increment;
  }
};

class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3WriteGnuplotAction
    : public WriteGnuplotAction {
 public:
  VertexSE3WriteGnuplotAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
};

#ifdef G2O_HAVE_OPENGL
class G2O_DEPRECATED_TYPES_SLAM3D_API VertexSE3DrawAction : public DrawAction {
 public:
  VertexSE3DrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);
  HyperGraphElementAction* _cacheDrawActions;

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty *_triangleX, *_triangleY;
};
#endif

}  // namespace deprecated
G2O_END_NAMESPACE

#endif
