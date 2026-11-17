// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d_addons/vertex_line2d.h>

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <g2o/stuff/macros.h>

#include <typeinfo>

G2O_START_NAMESPACE

VertexLine2D::VertexLine2D() : BaseVertex<2, Line2D>() {
  _estimate.setZero();
  p1Id = p2Id = -1;
}

bool VertexLine2D::read(std::istream& is) {
  is >> _estimate[0] >> _estimate[1] >> p1Id >> p2Id;
  return true;
}

bool VertexLine2D::write(std::ostream& os) const {
  os << estimate()(0) << " " << estimate()(1) << " " << p1Id << " " << p2Id;
  return os.good();
}

//   VertexLine2DWriteGnuplotAction::VertexLine2DWriteGnuplotAction():
//   WriteGnuplotAction(typeid(VertexLine2D).name()){}

//   HyperGraphElementAction*
//   VertexLine2DWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement*
//   element, HyperGraphElementAction::Parameters* params_){
//     if (typeid(*element).name()!=_typeName)
//       return 0;

//     WriteGnuplotAction::Parameters*
//     params=static_cast<WriteGnuplotAction::Parameters*>(params_); if
//     (!params->os){
//       std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified"
//       << std::endl; return false;
//     }

//     VertexLine2D* v =  static_cast<VertexLine2D*>(element);
//     *(params->os) << v->estimate().x() << " " << v->estimate().y() <<
//     std::endl; return this;
//   }

#ifdef G2O_HAVE_OPENGL
VertexLine2DDrawAction::VertexLine2DDrawAction()
    : DrawAction(typeid(VertexLine2D).name()) {}

bool VertexLine2DDrawAction::refreshPropertyPtrs(
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

HyperGraphElementAction* VertexLine2DDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  VertexLine2D* that = dynamic_cast<VertexLine2D*>(element);
  glPushAttrib(GL_CURRENT_BIT | GL_BLEND);
  if (_pointSize != nullptr) {
    glPointSize(_pointSize->value());
  }
  Vector2D n(cos(that->theta()), sin(that->theta()));
  Vector2D pmiddle = n * that->rho();
  Vector2D t(-n.y(), n.x());
  double l1, l2 = 10;
  VertexPointXY *vp1 = nullptr, *vp2 = nullptr;
  vp1 = dynamic_cast<VertexPointXY*>(that->graph()->vertex(that->p1Id));
  vp2 = dynamic_cast<VertexPointXY*>(that->graph()->vertex(that->p2Id));

  glColor4f(0.8f, 0.5f, 0.3f, 0.3f);
  if ((vp1 != nullptr) && (vp2 != nullptr)) {
    glColor4f(0.8f, 0.5f, 0.3f, 0.7f);
  } else if ((vp1 != nullptr) || (vp2 != nullptr)) {
    glColor4f(0.8f, 0.5f, 0.3f, 0.5f);
  }

  if (vp1 != nullptr) {
    glColor4f(0.8f, 0.5f, 0.3f, 0.7f);
    l1 = t.dot(vp1->estimate() - pmiddle);
  }
  if (vp2 != nullptr) {
    glColor4f(0.8f, 0.5f, 0.3f, 0.7f);
    l2 = t.dot(vp2->estimate() - pmiddle);
  }
  Vector2D p1 = pmiddle + t * l1;
  Vector2D p2 = pmiddle + t * l2;
  glBegin(GL_LINES);
  glVertex3f((float)p1.x(), p1.y(), 0.f);
  glVertex3f((float)p2.x(), p2.y(), 0.f);
  glEnd();
  glPopAttrib();
  return this;
}
#endif

G2O_END_NAMESPACE
