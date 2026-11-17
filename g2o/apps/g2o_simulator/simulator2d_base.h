// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SIMULATOR2D_BASE_H_
#define G2O_SIMULATOR2D_BASE_H_

#include <g2o/types/slam2d/types_slam2d.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>
#include "simulator.h"

G2O_START_NAMESPACE

  typedef WorldObject<VertexSE2> WorldObjectSE2;

  typedef WorldObject<VertexPointXY> WorldObjectPointXY;

  typedef WorldObject<VertexSegment2D> WorldObjectSegment2D;

  typedef Robot<WorldObjectSE2>  Robot2D;

G2O_END_NAMESPACE

#endif
