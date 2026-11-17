// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_CAMERA_PARAMETERS_H_
#define G2O_CAMERA_PARAMETERS_H_

#include <g2o/prefix.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>

G2O_START_NAMESPACE
/**
 * \brief parameters for a camera
 */
class G2O_TYPES_SLAM3D_API ParameterCamera : public ParameterSE3Offset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ParameterCamera();
  void setKcam(double fx, double fy, double cx, double cy);
  void setOffset(const Isometry3D& offset_ = Isometry3D::Identity());

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  const Matrix3D& Kcam() const { return _Kcam; }
  const Matrix3D& invKcam() const { return _invKcam; }
  const Matrix3D& Kcam_inverseOffsetR() const { return _Kcam_inverseOffsetR; }

 protected:
  Matrix3D _Kcam;
  Matrix3D _invKcam;
  Matrix3D _Kcam_inverseOffsetR;
};

class G2O_TYPES_SLAM3D_API CacheCamera : public CacheSE3Offset {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  //! parameters of the camera
  const ParameterCamera* camParams() const { return params; }
  //! return the world to image transform
  const Affine3D& w2i() const { return _w2i; }

 protected:
  virtual void updateImpl();
  virtual bool resolveDependancies();
  Affine3D _w2i;  ///< world to image transform
  ParameterCamera* params{};
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_SLAM3D_API CacheCameraDrawAction : public DrawAction {
 public:
  CacheCameraDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  FloatProperty *_cameraZ{}, *_cameraSide{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
