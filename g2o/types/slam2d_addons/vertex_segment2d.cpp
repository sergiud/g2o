// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d_addons/vertex_segment2d.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <g2o/stuff/macros.h>

#include <typeinfo>

G2O_START_NAMESPACE

VertexSegment2D::VertexSegment2D() : BaseVertex<4, Vector4D>() {
  _estimate.setZero();
}

bool VertexSegment2D::read(std::istream& is) {
  for (size_t i = 0; i < 4; i++) is >> _estimate[i];
  return true;
}

bool VertexSegment2D::write(std::ostream& os) const {
  for (size_t i = 0; i < 4; i++) os << _estimate[i] << " ";
  return os.good();
}

VertexSegment2DWriteGnuplotAction::VertexSegment2DWriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexSegment2D).name()) {}

HyperGraphElementAction* VertexSegment2DWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  WriteGnuplotAction::Parameters* params =
      dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
  if (params->os == nullptr) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
              << std::endl;
    return nullptr;
  }

  VertexSegment2D* v = dynamic_cast<VertexSegment2D*>(element);
  *(params->os) << v->estimateP1().x() << " " << v->estimateP1().y()
                << std::endl;
  *(params->os) << v->estimateP2().x() << " " << v->estimateP2().y()
                << std::endl;
  *(params->os) << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
VertexSegment2DDrawAction::VertexSegment2DDrawAction()
    : DrawAction(typeid(VertexSegment2D).name()) {}

bool VertexSegment2DDrawAction::refreshPropertyPtrs(
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

HyperGraphElementAction* VertexSegment2DDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  VertexSegment2D* that = dynamic_cast<VertexSegment2D*>(element);
  glColor3f(0.8f, 0.5f, 0.3f);
  if (_pointSize != nullptr) {
    glPointSize(_pointSize->value());
  }
  glBegin(GL_LINES);
  glVertex3f((float)that->estimateP1().x(), (float)that->estimateP1().y(), 0.f);
  glVertex3f((float)that->estimateP2().x(), (float)that->estimateP2().y(), 0.f);
  glEnd();
  return this;
}
#endif

G2O_END_NAMESPACE
