// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TUTORIAL_EDGE_SE2_H
#define G2O_TUTORIAL_EDGE_SE2_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>
#include <g2o/examples/tutorial_slam2d/se2.h>
#include <g2o/examples/tutorial_slam2d/vertex_se2.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

namespace tutorial {

/**
 * \brief 2D edge between two Vertex2, i.e., the odometry
 */
class G2O_TUTORIAL_SLAM2D_API EdgeSE2
    : public BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2();

  void computeError() {
    const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
    const VertexSE2* v2 = dynamic_cast<const VertexSE2*>(_vertices[1]);
    SE2 delta =
        _inverseMeasurement * (v1->estimate().inverse() * v2->estimate());
    _error = delta.toVector();
  }

  void setMeasurement(const SE2& m) {
    _measurement = m;
    _inverseMeasurement = m.inverse();
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

 protected:
  SE2 _inverseMeasurement;
};

}  // namespace tutorial

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
