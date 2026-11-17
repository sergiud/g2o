// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_
#define G2O_VERTEX_SE3_OFFSET_PARAMETERS_H_

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>

#include <Eigen/Geometry>

G2O_START_NAMESPACE

class VertexSE3;

/**
 * \brief offset for an SE3
 */
class G2O_TYPES_SLAM3D_API ParameterSE3Offset : public Parameter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ParameterSE3Offset();

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  /**
   * update the offset to a new value.
   * re-calculates the different representations, e.g., the rotation matrix
   */
  void setOffset(const Isometry3D& offset_ = Isometry3D::Identity());

  //! rotation of the offset as 3x3 rotation matrix
  const Isometry3D& offset() const { return _offset; }

  //! rotation of the inverse offset as 3x3 rotation matrix
  const Isometry3D& inverseOffset() const { return _inverseOffset; }

 protected:
  Isometry3D _offset;
  Isometry3D _inverseOffset;
};

/**
 * \brief caching the offset related to a vertex
 */
class G2O_TYPES_SLAM3D_API CacheSE3Offset : public Cache {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  CacheSE3Offset();
  virtual void updateImpl();

  const ParameterSE3Offset* offsetParam() const { return _offsetParam; }
  void setOffsetParam(ParameterSE3Offset* offsetParam);

  const Isometry3D& w2n() const { return _w2n; }
  const Isometry3D& n2w() const { return _n2w; }
  const Isometry3D& w2l() const { return _w2l; }

 protected:
  ParameterSE3Offset* _offsetParam;  ///< the parameter connected to the cache
  Isometry3D _w2n;
  Isometry3D _n2w;
  Isometry3D _w2l;

 protected:
  virtual bool resolveDependancies();
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM3D_API CacheSE3OffsetDrawAction : public DrawAction {
 public:
  CacheSE3OffsetDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty* _cubeSide{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
