// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/logger.h>
#include <g2o/types/slam2d/vertex_point_xy.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <g2o/stuff/macros.h>

#include <typeinfo>

G2O_START_NAMESPACE

VertexPointXY::VertexPointXY() : BaseVertex<2, Vector2D>() {
  _estimate.setZero();
}

bool VertexPointXY::read(std::istream& is) {
  is >> _estimate[0] >> _estimate[1];
  return !is.fail();
}

bool VertexPointXY::write(std::ostream& os) const {
  os << estimate()(0) << " " << estimate()(1);
  return os.good();
}

VertexPointXYWriteGnuplotAction::VertexPointXYWriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexPointXY).name()) {}

HyperGraphElementAction* VertexPointXYWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  WriteGnuplotAction::Parameters* params =
      dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
  if (params->os == nullptr) {
    ngx::Log(ngx::Warning, "on valid os specified");
    return nullptr;
  }

  VertexPointXY* v = dynamic_cast<VertexPointXY*>(element);
  *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
VertexPointXYDrawAction::VertexPointXYDrawAction()
    : DrawAction(typeid(VertexPointXY).name()) {}

bool VertexPointXYDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _pointSize = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::POINT_SIZE", 1.);
  } else {
    _pointSize = nullptr;
  }
  return true;
}

HyperGraphElementAction* VertexPointXYDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params) {
  if (typeid(*element).name() != _typeName) return nullptr;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;
  VertexPointXY* that = dynamic_cast<VertexPointXY*>(element);

  glPushMatrix();
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glColor3f(LANDMARK_VERTEX_COLOR);
  float ps = _pointSize != nullptr ? _pointSize->value() : 1.0f;
  glTranslatef((float)that->estimate()(0), (float)that->estimate()(1), 0.0f);
  opengl::drawPoint(ps);
  glPopAttrib();
  drawCache(that->cacheContainer(), params);
  drawUserData(that->userData(), params);
  glPopMatrix();
  return this;
}
#endif

G2O_END_NAMESPACE
