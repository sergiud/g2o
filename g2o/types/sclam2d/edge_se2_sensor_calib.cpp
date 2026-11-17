// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/sclam2d/edge_se2_sensor_calib.h>
#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#endif
G2O_START_NAMESPACE

EdgeSE2SensorCalib::EdgeSE2SensorCalib() : BaseMultiEdge<3, SE2>() {
  resize(3);
}

void EdgeSE2SensorCalib::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) {
  (void)to;
  VertexSE2* vi = dynamic_cast<VertexSE2*>(_vertices[0]);
  VertexSE2* vj = dynamic_cast<VertexSE2*>(_vertices[1]);
  VertexSE2* l = dynamic_cast<VertexSE2*>(_vertices[2]);
  if (from.count(l) == 0) return;
  if (from.count(vi) == 1) {
    vj->setEstimate(vi->estimate() * l->estimate() * measurement() *
                    l->estimate().inverse());
  } else {
    vi->setEstimate(vj->estimate() * l->estimate() * _inverseMeasurement *
                    l->estimate().inverse());
  }
}

bool EdgeSE2SensorCalib::read(std::istream& is) {
  Vector3D p;
  is >> p(0) >> p(1) >> p(2);
  _measurement.fromVector(p);
  _inverseMeasurement = measurement().inverse();
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return true;
}

bool EdgeSE2SensorCalib::write(std::ostream& os) const {
  Vector3D p = measurement().toVector();
  os << p(0) << " " << p(1) << " " << p(2);
  for (int i = 0; i < information().rows(); ++i)
    for (int j = i; j < information().cols(); ++j)
      os << " " << information()(i, j);
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
EdgeSE2SensorCalibDrawAction::EdgeSE2SensorCalibDrawAction()
    : DrawAction(typeid(EdgeSE2SensorCalib).name()) {}

HyperGraphElementAction* EdgeSE2SensorCalibDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters*) {
  if (typeid(*element).name() != _typeName) return nullptr;
  EdgeSE2SensorCalib* e = dynamic_cast<EdgeSE2SensorCalib*>(element);
  VertexSE2* fromEdge = dynamic_cast<VertexSE2*>(e->vertex(0));
  VertexSE2* toEdge = dynamic_cast<VertexSE2*>(e->vertex(1));
  glColor3f(0.5, 0.5, 1.0);
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
