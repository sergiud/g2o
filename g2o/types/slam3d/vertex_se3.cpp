// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/types/slam3d/vertex_se3.h>
#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <g2o/core/cache.h>

#include <iostream>

using namespace Eigen;

G2O_START_NAMESPACE

VertexSE3::VertexSE3() : BaseVertex<6, Isometry3D>(), _numOplusCalls(0) {
  setToOriginImpl();
  updateCache();
}

bool VertexSE3::read(std::istream& is) {
  Vector7d est;
  for (int i = 0; i < 7; i++) is >> est[i];
  setEstimate(internal::fromVectorQT(est));
  return true;
}

bool VertexSE3::write(std::ostream& os) const {
  Vector7d est = internal::toVectorQT(_estimate);
  for (int i = 0; i < 7; i++) os << est[i] << " ";
  return os.good();
}

VertexSE3WriteGnuplotAction::VertexSE3WriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexSE3).name()) {}

HyperGraphElementAction* VertexSE3WriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  WriteGnuplotAction::Parameters* params =
      dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
  if (params->os == nullptr) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified"
              << std::endl;
    return nullptr;
  }

  VertexSE3* v = dynamic_cast<VertexSE3*>(element);
  Vector6d est = internal::toVectorMQT(v->estimate());
  for (int i = 0; i < 6; i++) *(params->os) << est[i] << " ";
  *(params->os) << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
void drawTriangle(float xSize, float ySize) {
  Vector3F p[3];
  glBegin(GL_TRIANGLES);
  p[0] << 0., 0., 0.;
  p[1] << -xSize, ySize, 0.;
  p[2] << -xSize, -ySize, 0.;
  for (int i = 1; i < 2; ++i) {
    Vector3F normal = (p[i] - p[0]).cross(p[i + 1] - p[0]);
    glNormal3f(normal.x(), normal.y(), normal.z());
    glVertex3f(p[0].x(), p[0].y(), p[0].z());
    glVertex3f(p[i].x(), p[i].y(), p[i].z());
    glVertex3f(p[i + 1].x(), p[i + 1].y(), p[i + 1].z());
  }
  glEnd();
}

VertexSE3DrawAction::VertexSE3DrawAction()
    : DrawAction(typeid(VertexSE3).name()) {
  _cacheDrawActions = nullptr;
}

bool VertexSE3DrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _triangleX = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::TRIANGLE_X", .2f);
    _triangleY = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::TRIANGLE_Y", .05f);
  } else {
    _triangleX = nullptr;
    _triangleY = nullptr;
  }
  return true;
}

HyperGraphElementAction* VertexSE3DrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params_);

  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  VertexSE3* that = dynamic_cast<VertexSE3*>(element);

  glColor3f(POSE_VERTEX_COLOR);
  glPushMatrix();
  glMultMatrixd(that->estimate().matrix().data());
  opengl::drawArrow2D(_triangleX->value(), _triangleY->value(),
                      _triangleX->value() * .3f);
  drawCache(that->cacheContainer(), params_);
  drawUserData(that->userData(), params_);
  glPopMatrix();
  return this;
}
#endif

G2O_END_NAMESPACE
