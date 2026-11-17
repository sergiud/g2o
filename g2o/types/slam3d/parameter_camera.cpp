// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/isometry3d_gradients.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o/types/slam3d/parameter_camera.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

using namespace std;

G2O_START_NAMESPACE

ParameterCamera::ParameterCamera() {
  setId(-1);
  setKcam(1, 1, 0.5, 0.5);
  setOffset();
}

void ParameterCamera::setOffset(const Isometry3D& offset_) {
  ParameterSE3Offset::setOffset(offset_);
  _Kcam_inverseOffsetR = _Kcam * inverseOffset().rotation();
}

void ParameterCamera::setKcam(double fx, double fy, double cx, double cy) {
  _Kcam.setZero();
  _Kcam(0, 0) = fx;
  _Kcam(1, 1) = fy;
  _Kcam(0, 2) = cx;
  _Kcam(1, 2) = cy;
  _Kcam(2, 2) = 1.0;
  _invKcam = _Kcam.inverse();
  _Kcam_inverseOffsetR = _Kcam * inverseOffset().rotation();
}

bool ParameterCamera::read(std::istream& is) {
  Vector7d off;
  for (int i = 0; i < 7; i++) is >> off[i];
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4D::MapType(off.data() + 3).normalize();
  setOffset(internal::fromVectorQT(off));
  double fx, fy, cx, cy;
  is >> fx >> fy >> cx >> cy;
  setKcam(fx, fy, cx, cy);
  return is.good();
}

bool ParameterCamera::write(std::ostream& os) const {
  Vector7d off = internal::toVectorQT(_offset);
  for (int i = 0; i < 7; i++) os << off[i] << " ";
  os << _Kcam(0, 0) << " ";
  os << _Kcam(1, 1) << " ";
  os << _Kcam(0, 2) << " ";
  os << _Kcam(1, 2) << " ";
  return os.good();
}

bool CacheCamera::resolveDependancies() {
  if (!CacheSE3Offset::resolveDependancies()) return false;
  params = dynamic_cast<ParameterCamera*>(_parameters[0]);
  return params != nullptr;
}

void CacheCamera::updateImpl() {
  CacheSE3Offset::updateImpl();
  _w2i.matrix().topLeftCorner<3, 4>() =
      params->Kcam() * w2n().matrix().topLeftCorner<3, 4>();
}

#ifdef G2O_HAVE_OPENGL

CacheCameraDrawAction::CacheCameraDrawAction()
    : DrawAction(typeid(CacheCamera).name()) {
  _previousParams = (DrawAction::Parameters*)0x42;
  refreshPropertyPtrs(nullptr);
}

bool CacheCameraDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _cameraZ = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::CAMERA_Z", .05f);
    _cameraSide = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::CAMERA_SIDE", .05f);

  } else {
    _cameraZ = nullptr;
    _cameraSide = nullptr;
  }
  return true;
}

HyperGraphElementAction* CacheCameraDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params) {
  if (typeid(*element).name() != _typeName) return nullptr;
  CacheCamera* that = dynamic_cast<CacheCamera*>(element);
  refreshPropertyPtrs(params);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  glPushAttrib(GL_COLOR);
  glColor3f(POSE_PARAMETER_COLOR);
  glPushMatrix();
  glMultMatrixd(that->camParams()->offset().data());
  glRotatef(180.0f, 0.0f, 1.0f, 0.0f);
  opengl::drawPyramid(_cameraSide->value(), _cameraZ->value());
  glPopMatrix();
  glPopAttrib();
  return this;
}
#endif

G2O_END_NAMESPACE
