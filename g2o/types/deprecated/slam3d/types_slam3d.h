// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DEPRECATED_TYPES_SLAM3D_
#define G2O_DEPRECATED_TYPES_SLAM3D_

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/hyper_graph_action.h>

#define THREE_D_TYPES_ANALYTIC_JACOBIAN

#include "vertex_se3_quat.h"
#include "edge_se3_quat.h"
#include "vertex_pointxyz.h"

#include "parameter_se3_offset.h"
#include "edge_se3_pointxyz.h"
#include "edge_se3_offset.h"

#include "parameter_camera.h"
#include "edge_se3_pointxyz_disparity.h"
#include "edge_se3_pointxyz_depth.h"
#include "edge_se3_prior.h"

#endif
