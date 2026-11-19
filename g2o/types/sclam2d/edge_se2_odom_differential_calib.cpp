// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/sclam2d/edge_se2_odom_differential_calib.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#endif

G2O_START_NAMESPACE

EdgeSE2OdomDifferentialCalib::EdgeSE2OdomDifferentialCalib()
    : BaseMultiEdge<3, VelocityMeasurement>() {
  resize(3);
}

bool EdgeSE2OdomDifferentialCalib::read(std::istream& is) {
  double vl, vr, dt;
  is >> vl >> vr >> dt;
  VelocityMeasurement vm(vl, vr, dt);
  setMeasurement(vm);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return !is.fail();
}

bool EdgeSE2OdomDifferentialCalib::write(std::ostream& os) const {
  os << measurement().vl() << " " << measurement().vr() << " "
     << measurement().dt();
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j)
      os << " " << information()(i, j);
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2OdomDifferentialCalibDrawAction::EdgeSE2OdomDifferentialCalibDrawAction()
    : DrawAction(typeid(EdgeSE2OdomDifferentialCalib).name()) {}

HyperGraphElementAction* EdgeSE2OdomDifferentialCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters*) {
  if (typeid(*element).name() != _typeName) return nullptr;
  EdgeSE2OdomDifferentialCalib* e =
      dynamic_cast<EdgeSE2OdomDifferentialCalib*>(element);
  VertexSE2* fromEdge = dynamic_cast<VertexSE2*>(e->vertex(0));
  VertexSE2* toEdge = dynamic_cast<VertexSE2*>(e->vertex(1));
  glColor3f(0.5f, 0.5f, 0.5f);
  glPushAttrib(GL_ENABLE_BIT);
  glDisable(GL_LIGHTING);
  glBegin(GL_LINES);
  glVertex3f((float)fromEdge->estimate().translation().x(),
             (float)fromEdge->estimate().translation().y(), 0.f);
  glVertex3f((float)toEdge->estimate().translation().x(),
             (float)toEdge->estimate().translation().y(), 0.f);
  glEnd();
  glPopAttrib();
  return this;
}
#endif

G2O_END_NAMESPACE
