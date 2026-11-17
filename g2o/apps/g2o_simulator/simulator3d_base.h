// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SIMULATOR3D_BASE_H_
#define G2O_SIMULATOR3D_BASE_H_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include "simulator.h"

G2O_START_NAMESPACE

typedef WorldObject<VertexSE3> WorldObjectSE3;

typedef WorldObject<VertexPointXYZ> WorldObjectTrackXYZ;

typedef WorldObject<VertexLine3D> WorldObjectLine3D;

typedef Robot<WorldObjectSE3> Robot3D;

G2O_END_NAMESPACE

#endif
