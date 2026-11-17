// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/opengl_wrapper.h>
#include <g2o/types/slam3d_addons/vertex_line3d.h>

G2O_START_NAMESPACE

VertexLine3D::VertexLine3D() { color << 1.0, 0.5, 0.0; }

bool VertexLine3D::read(std::istream& is) {
  Vector6d lv;
  for (int i = 0; i < 6; ++i) {
    is >> lv[i];
  }
  setEstimate(Line3D(lv));
  return true;
}

bool VertexLine3D::write(std::ostream& os) const {
  Vector6d lv = _estimate;
  for (int i = 0; i < 6; ++i) {
    os << lv[i] << " ";
  }
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
VertexLine3DDrawAction::VertexLine3DDrawAction()
    : DrawAction(typeid(VertexLine3D).name()) {}

bool VertexLine3DDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) {
    return false;
  }
  if (_previousParams != nullptr) {
    _lineLength = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::LINE_LENGTH", 15);
    _lineWidth = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::LINE_WIDTH", 5);
  } else {
    _lineLength = nullptr;
    _lineWidth = nullptr;
  }
  return true;
}

HyperGraphElementAction* VertexLine3DDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) {
    return nullptr;
  }

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) {
    return this;
  }

  if ((_show != nullptr) && !_show->value()) {
    return this;
  }

  VertexLine3D* that = dynamic_cast<VertexLine3D*>(element);
  Line3D line = that->estimate();
  line.normalize();
  Eigen::Vector3d direction = line.d();
  Eigen::Vector3d npoint = line.d().cross(line.w());
  glPushMatrix();
  glColor3f(float(that->color(0)), float(that->color(1)),
            float(that->color(2)));
  if ((_lineLength != nullptr) && (_lineWidth != nullptr)) {
    glLineWidth(float(_lineWidth->value()));
    glBegin(GL_LINES);
    glNormal3f(float(npoint.x()), float(npoint.y()), float(npoint.z()));
    glVertex3f(float(npoint.x() - direction.x() * _lineLength->value() / 2),
               float(npoint.y() - direction.y() * _lineLength->value() / 2),
               float(npoint.z() - direction.z() * _lineLength->value() / 2));
    glVertex3f(float(npoint.x() + direction.x() * _lineLength->value() / 2),
               float(npoint.y() + direction.y() * _lineLength->value() / 2),
               float(npoint.z() + direction.z() * _lineLength->value() / 2));
    glEnd();
  }
  glPopMatrix();

  return this;
}
#endif

G2O_END_NAMESPACE
