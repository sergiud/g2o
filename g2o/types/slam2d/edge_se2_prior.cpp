// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d/edge_se2_prior.h>

G2O_START_NAMESPACE

EdgeSE2Prior::EdgeSE2Prior() : BaseUnaryEdge<3, SE2, VertexSE2>() {}

void EdgeSE2Prior::initialEstimate(const OptimizableGraph::VertexSet& from,
                                   OptimizableGraph::Vertex* to) {
  assert(from.size() == 0);
  (void)from;
  (void)to;
  VertexSE2* v1 = dynamic_cast<VertexSE2*>(_vertices[0]);
  v1->setEstimate(_measurement);
}

bool EdgeSE2Prior::read(std::istream& is) {
  Vector3D p;
  is >> p[0] >> p[1] >> p[2];
  setMeasurement(p);
  _inverseMeasurement = _measurement.inverse();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) {
      is >> information()(i, j);
      if (i != j) information()(j, i) = information()(i, j);
    }
  return !is.fail();
}

bool EdgeSE2Prior::write(std::ostream& os) const {
  Vector3D p = measurement().toVector();
  os << p.x() << " " << p.y() << " " << p.z();
  for (int i = 0; i < 3; ++i)
    for (int j = i; j < 3; ++j) os << " " << information()(i, j);
  return os.good();
}

void EdgeSE2Prior::setMeasurement(const SE2& m) {
  _measurement = m;
  _inverseMeasurement = m.inverse();
}

bool EdgeSE2Prior::setMeasurementData(const double* d) {
  _measurement = SE2(d[0], d[1], d[2]);
  _inverseMeasurement = _measurement.inverse();
  return true;
}

G2O_END_NAMESPACE
