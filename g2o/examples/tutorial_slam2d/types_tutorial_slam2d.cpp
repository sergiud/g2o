// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "types_tutorial_slam2d.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

#include <iostream>


#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>

G2O_START_NAMESPACE
  namespace tutorial {

  G2O_REGISTER_TYPE_GROUP(tutorial_slam2d);

  G2O_REGISTER_TYPE(TUTORIAL_VERTEX_SE2, VertexSE2);
  G2O_REGISTER_TYPE(TUTORIAL_VERTEX_POINT_XY, VertexPointXY);

  G2O_REGISTER_TYPE(TUTORIAL_PARAMS_SE2_OFFSET, ParameterSE2Offset);

  G2O_REGISTER_TYPE(TUTORIAL_CACHE_SE2_OFFSET, CacheSE2Offset);

  G2O_REGISTER_TYPE(TUTORIAL_EDGE_SE2, EdgeSE2);
  G2O_REGISTER_TYPE(TUTORIAL_EDGE_SE2_POINT_XY, EdgeSE2PointXY);
  } // namespace tutorial
G2O_END_NAMESPACE
