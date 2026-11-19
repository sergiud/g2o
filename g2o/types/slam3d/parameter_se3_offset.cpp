// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/dquat2mat.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_se3.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

G2O_START_NAMESPACE

ParameterSE3Offset::ParameterSE3Offset() { setOffset(); }

void ParameterSE3Offset::setOffset(const Isometry3D& offset_) {
  _offset = offset_;
  _inverseOffset = _offset.inverse();
}

bool ParameterSE3Offset::read(std::istream& is) {
  Vector7d off;
  for (int i = 0; i < 7; i++) {
    is >> off[i];
  }
  // normalize the quaternion to recover numerical precision lost by storing as
  // human readable text
  Vector4D::MapType(off.data() + 3).normalize();
  setOffset(internal::fromVectorQT(off));
  return !is.fail();
}

bool ParameterSE3Offset::write(std::ostream& os) const {
  Vector7d off = internal::toVectorQT(_offset);
  for (int i = 0; i < 7; i++) os << off[i] << " ";
  return os.good();
}

CacheSE3Offset::CacheSE3Offset() : Cache(), _offsetParam(nullptr) {}

bool CacheSE3Offset::resolveDependancies() {
  _offsetParam = dynamic_cast<ParameterSE3Offset*>(_parameters[0]);
  return _offsetParam != nullptr;
}

void CacheSE3Offset::updateImpl() {
  const VertexSE3* v = dynamic_cast<const VertexSE3*>(vertex());
  _n2w = v->estimate() * _offsetParam->offset();
  _w2n = _n2w.inverse();
  _w2l = v->estimate().inverse();
}

void CacheSE3Offset::setOffsetParam(ParameterSE3Offset* offsetParam) {
  _offsetParam = offsetParam;
}

#ifdef G2O_HAVE_OPENGL
CacheSE3OffsetDrawAction::CacheSE3OffsetDrawAction()
    : DrawAction(typeid(CacheSE3Offset).name()) {
  _previousParams = (DrawAction::Parameters*)0x42;
  refreshPropertyPtrs(nullptr);
}

bool CacheSE3OffsetDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _cubeSide = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::CUBE_SIDE", .05f);
  } else {
    _cubeSide = nullptr;
  }
  return true;
}

HyperGraphElementAction* CacheSE3OffsetDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  CacheSE3Offset* that = dynamic_cast<CacheSE3Offset*>(element);
  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;
  float cs = _cubeSide != nullptr ? _cubeSide->value() : 1.0f;
  glPushAttrib(GL_COLOR);
  glColor3f(POSE_PARAMETER_COLOR);
  glPushMatrix();
  glMultMatrixd(that->offsetParam()->offset().data());
  opengl::drawBox(cs, cs, cs);
  glPopMatrix();
  glPopAttrib();
  return this;
}
#endif

G2O_END_NAMESPACE
