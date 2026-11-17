// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d/edge_se2_pointxy_calib.h>

G2O_START_NAMESPACE

EdgeSE2PointXYCalib::EdgeSE2PointXYCalib() : BaseMultiEdge<2, Vector2D>() {
  resize(3);
}

void EdgeSE2PointXYCalib::initialEstimate(
    const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* /*to*/) {
  assert(from.size() == 1 && from.count(_vertices[0]) == 1 &&
         "Can not initialize VertexSE2 position by VertexPointXY");

  if (from.count(_vertices[0]) != 1) return;
  VertexSE2* vi = dynamic_cast<VertexSE2*>(_vertices[0]);
  VertexPointXY* vj = dynamic_cast<VertexPointXY*>(_vertices[1]);
  vj->setEstimate(vi->estimate() * _measurement);
}

bool EdgeSE2PointXYCalib::read(std::istream& is) {
  is >> _measurement[0] >> _measurement[1];
  is >> information()(0, 0) >> information()(0, 1) >> information()(1, 1);
  information()(1, 0) = information()(0, 1);
  return true;
}

bool EdgeSE2PointXYCalib::write(std::ostream& os) const {
  os << measurement()[0] << " " << measurement()[1] << " ";
  os << information()(0, 0) << " " << information()(0, 1) << " "
     << information()(1, 1);
  return os.good();
}

G2O_END_NAMESPACE
