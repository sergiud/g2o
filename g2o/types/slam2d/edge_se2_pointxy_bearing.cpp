// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2015-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/logger.h>
#include <g2o/types/slam2d/edge_se2_pointxy_bearing.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

G2O_START_NAMESPACE

EdgeSE2PointXYBearing::EdgeSE2PointXYBearing() {}

void EdgeSE2PointXYBearing::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/) {
  assert(from.size() == 1 && from.count(_vertices[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexPointXY");

  if (from.count(_vertices[0]) != 1) return;
  double r = 2.;
  const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
  VertexPointXY* l2 = dynamic_cast<VertexPointXY*>(_vertices[1]);
  SE2 t = v1->estimate();
  t.setRotation(t.rotation() * Eigen::Rotation2Dd(_measurement));
  Vector2D vr(r, 0.);
  l2->setEstimate(t * vr);
}

#ifndef NUMERIC_JACOBIAN_TWO_D_TYPES
void EdgeSE2PointXYBearing::linearizeOplus() {
  const VertexSE2* vi = static_cast<const VertexSE2*>(_vertices[0]);
  const VertexPointXY* vj = static_cast<const VertexPointXY*>(_vertices[1]);
  const double x1 = vi->estimate().translation()[0];
  const double y1 = vi->estimate().translation()[1];
  const double x2 = vj->estimate()[0];
  const double y2 = vj->estimate()[1];

  // squared distance between the pose and the point
  const double squaredDistance = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);

  _jacobianOplusXi(0, 0) = (y1 - y2) / squaredDistance;
  _jacobianOplusXi(0, 1) = (x2 - x1) / squaredDistance;
  _jacobianOplusXi(0, 2) = 1;

  _jacobianOplusXj(0, 0) = (y2 - y1) / squaredDistance;
  _jacobianOplusXj(0, 1) = (x1 - x2) / squaredDistance;
}
#endif

bool EdgeSE2PointXYBearing::read(std::istream& is) {
  is >> _measurement >> information()(0, 0);
  return true;
}

bool EdgeSE2PointXYBearing::write(std::ostream& os) const {
  os << measurement() << " " << information()(0, 0);
  return os.good();
}

EdgeSE2PointXYBearingWriteGnuplotAction::
    EdgeSE2PointXYBearingWriteGnuplotAction()
    : WriteGnuplotAction(typeid(EdgeSE2PointXYBearing).name()) {}

HyperGraphElementAction* EdgeSE2PointXYBearingWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  WriteGnuplotAction::Parameters* params =
      dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
  if (params->os == nullptr) {
    ngx::Log(ngx::Warning, "on valid os specified");
    return nullptr;
  }

  EdgeSE2PointXYBearing* e = dynamic_cast<EdgeSE2PointXYBearing*>(element);
  VertexSE2* fromEdge = dynamic_cast<VertexSE2*>(e->vertex(0));
  VertexPointXY* toEdge = dynamic_cast<VertexPointXY*>(e->vertex(1));
  *(params->os) << fromEdge->estimate().translation().x() << " "
                << fromEdge->estimate().translation().y() << " "
                << fromEdge->estimate().rotation().angle() << std::endl;
  *(params->os) << toEdge->estimate().x() << " " << toEdge->estimate().y()
                << std::endl;
  *(params->os) << std::endl;
  return this;
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2PointXYBearingDrawAction::EdgeSE2PointXYBearingDrawAction()
    : DrawAction(typeid(EdgeSE2PointXYBearing).name()) {}

HyperGraphElementAction* EdgeSE2PointXYBearingDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) return this;

  if ((_show != nullptr) && !_show->value()) return this;

  EdgeSE2PointXYBearing* e = dynamic_cast<EdgeSE2PointXYBearing*>(element);
  VertexSE2* from = dynamic_cast<VertexSE2*>(e->vertex(0));
  VertexPointXY* to = dynamic_cast<VertexPointXY*>(e->vertex(1));
  if (from == nullptr) return this;
  double guessRange = 5;
  double theta = e->measurement();
  Vector2D p(cos(theta) * guessRange, sin(theta) * guessRange);
  glPushAttrib(GL_ENABLE_BIT | GL_LIGHTING | GL_COLOR);
  glDisable(GL_LIGHTING);
  if (to == nullptr) {
    p = from->estimate() * p;
    glColor3f(LANDMARK_EDGE_GHOST_COLOR);
    glPushAttrib(GL_POINT_SIZE);
    glPointSize(3);
    glBegin(GL_POINTS);
    glVertex3f((float)p.x(), (float)p.y(), 0.f);
    glEnd();
    glPopAttrib();
  } else {
    p = to->estimate();
    glColor3f(LANDMARK_EDGE_COLOR);
  }
  glBegin(GL_LINES);
  glVertex3f((float)from->estimate().translation().x(),
             (float)from->estimate().translation().y(), 0.f);
  glVertex3f((float)p.x(), (float)p.y(), 0.f);
  glEnd();
  glPopAttrib();
  return this;
}
#endif

G2O_END_NAMESPACE
