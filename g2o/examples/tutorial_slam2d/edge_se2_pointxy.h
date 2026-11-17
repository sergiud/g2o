// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TUTORIAL_EDGE_SE2_POINT_XY_H
#define G2O_TUTORIAL_EDGE_SE2_POINT_XY_H

#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>
#include <g2o/prefix.h>

#include "parameter_se2_offset.h"
#include "types_tutorial_slam2d.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

G2O_START_NAMESPACE

namespace tutorial {

class ParameterSE2Offset;
class CacheSE2Offset;

class G2O_TUTORIAL_SLAM2D_API EdgeSE2PointXY
    : public BaseBinaryEdge<2, Eigen::Vector2d, VertexSE2, VertexPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeSE2PointXY();

  void computeError();

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

 protected:
  ParameterSE2Offset* _sensorOffset;
  CacheSE2Offset* _sensorCache;

  virtual bool resolveCaches();
};

}  // namespace tutorial
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
