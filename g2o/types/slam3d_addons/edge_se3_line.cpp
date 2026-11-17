// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/opengl_wrapper.h>
#include <g2o/types/slam3d_addons/edge_se3_line.h>

G2O_START_NAMESPACE

EdgeSE3Line3D::EdgeSE3Line3D() {
  information().setIdentity();
  cache = nullptr;
  offsetParam = nullptr;
  resizeParameters(1);
  installParameter(offsetParam, 0);
  color << 0.0, 0.5, 1.0;
}

bool EdgeSE3Line3D::read(std::istream& is) {
  int pId;
  is >> pId;
  setParameterId(0, pId);

  for (int i = 0; i < 6; ++i) {
    is >> _measurement(i);
  }
  information().setZero();
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < 4; ++j) {
      is >> information()(i, j);
      information()(i, j) = information()(j, i);
    }
  }
  return is.good();
}

bool EdgeSE3Line3D::write(std::ostream& os) const {
  os << offsetParam->id() << " ";
  for (int i = 0; i < 6; ++i) {
    os << _measurement(i) << " ";
  }
  for (int i = 0; i < 4; ++i) {
    for (int j = i; j < 4; ++j) {
      os << information()(i, j) << " ";
    }
  }
  return os.good();
}

void EdgeSE3Line3D::computeError() {
  const VertexSE3* se3Vertex = dynamic_cast<const VertexSE3*>(_vertices[0]);
  const VertexLine3D* lineVertex =
      dynamic_cast<const VertexLine3D*>(_vertices[1]);
  const Line3D& line = lineVertex->estimate();
  Line3D localLine = se3Vertex->estimate().inverse() * line;
  _error = localLine.ominus(_measurement);
}

bool EdgeSE3Line3D::resolveCaches() {
  ParameterVector pv(1);
  pv[0] = offsetParam;
  resolveCache(cache, dynamic_cast<OptimizableGraph::Vertex*>(_vertices[0]),
               "CACHE_SE3_OFFSET", pv);
  return cache != nullptr;
}

#ifdef G2O_HAVE_OPENGL
EdgeSE3Line3DDrawAction::EdgeSE3Line3DDrawAction()
    : DrawAction(typeid(EdgeSE3Line3D).name()) {}

bool EdgeSE3Line3DDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) {
    return false;
  }
  if (_previousParams != nullptr) {
    _lineLength = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::LINE_LENGTH", 4.0f);
    _lineWidth = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::LINE_WIDTH", 2.0f);
  } else {
    _lineLength = nullptr;
    _lineWidth = nullptr;
  }
  return true;
}

HyperGraphElementAction* EdgeSE3Line3DDrawAction::operator()(
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

  EdgeSE3Line3D* that = dynamic_cast<EdgeSE3Line3D*>(element);

  if (that == nullptr) {
    return this;
  }

  const VertexSE3* robot = dynamic_cast<const VertexSE3*>(that->vertex(0));
  const VertexLine3D* landmark =
      dynamic_cast<const VertexLine3D*>(that->vertex(1));

  if ((robot == nullptr) || (landmark == nullptr)) {
    return nullptr;
  }

  Line3D line = that->measurement();
  line.normalize();
  Eigen::Vector3d direction = line.d();
  Eigen::Vector3d npoint = line.d().cross(line.w());

  glPushMatrix();
  glMultMatrixd(robot->estimate().matrix().data());
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
