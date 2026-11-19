// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/opengl_wrapper.h>
#include <g2o/types/slam3d_addons/vertex_plane.h>

G2O_START_NAMESPACE

VertexPlane::VertexPlane() { color << .2, .2, .2; }

bool VertexPlane::read(std::istream& is) {
  Vector4D lv;
  for (int i = 0; i < 4; i++) is >> lv[i];
  setEstimate(Plane3D(lv));
  is >> color(0) >> color(1) >> color(2);
  return !is.fail();
}

bool VertexPlane::write(std::ostream& os) const {
  Vector4D lv = _estimate.toVector();
  for (int i = 0; i < 4; i++) {
    os << lv[i] << " ";
  }
  os << color(0) << " " << color(1) << " " << color(2) << " ";
  return os.good();
}

#ifdef G2O_HAVE_OPENGL

VertexPlaneDrawAction::VertexPlaneDrawAction()
    : DrawAction(typeid(VertexPlane).name()) {}

bool VertexPlaneDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _planeWidth = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::PLANE_WIDTH", 3);
    _planeHeight = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::PLANE_HEIGHT", 3);
  } else {
    _planeWidth = nullptr;
    _planeHeight = nullptr;
  }
  return true;
}

HyperGraphElementAction* VertexPlaneDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  VertexPlane* that = dynamic_cast<VertexPlane*>(element);
  double d = that->estimate().distance();
  double azimuth = Plane3D::azimuth(that->estimate().normal());
  double elevation = Plane3D::elevation(that->estimate().normal());
  glColor3f(float(that->color(0)), float(that->color(1)),
            float(that->color(2)));
  glPushMatrix();
  glRotatef(float(RAD2DEG(azimuth)), 0.f, 0.f, 1.f);
  glRotatef(float(RAD2DEG(elevation)), 0.f, -1.f, 0.f);
  glTranslatef(float(d), 0.f, 0.f);

  if ((_planeWidth != nullptr) && (_planeHeight != nullptr)) {
    glBegin(GL_QUADS);
    glNormal3f(-1.f, 0.f, 0.f);
    glVertex3f(0.f, -_planeWidth->value(), -_planeHeight->value());
    glVertex3f(0.f, _planeWidth->value(), -_planeHeight->value());
    glVertex3f(0.f, _planeWidth->value(), _planeHeight->value());
    glVertex3f(0.f, -_planeWidth->value(), _planeHeight->value());
    glEnd();
  }

  glPopMatrix();
  return this;
}
#endif

G2O_END_NAMESPACE
