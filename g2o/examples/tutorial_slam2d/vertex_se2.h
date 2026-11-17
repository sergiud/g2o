// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TUTORIAL_VERTEX_SE2_H
#define G2O_TUTORIAL_VERTEX_SE2_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>
#include <g2o/examples/tutorial_slam2d/se2.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE
namespace tutorial {

/**
 * \brief 2D pose Vertex, (x,y,theta)
 */
class G2O_TUTORIAL_SLAM2D_API VertexSE2 : public BaseVertex<3, SE2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexSE2();

  virtual void setToOriginImpl() { _estimate = SE2(); }

  virtual void oplusImpl(const double* update) {
    SE2 up(update[0], update[1], update[2]);
    _estimate *= up;
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

}  // namespace tutorial
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
