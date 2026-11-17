// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef G2O_TYPES_SLAM3D_
#define G2O_TYPES_SLAM3D_

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

#define THREE_D_TYPES_ANALYTIC_JACOBIAN

#include <g2o/types/icp/g2o_types_icp_api.h>
#include <g2o/types/slam3d/edge_pointxyz.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_lotsofxyz.h>
#include <g2o/types/slam3d/edge_se3_offset.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/edge_se3_pointxyz_depth.h>
#include <g2o/types/slam3d/edge_se3_pointxyz_disparity.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#endif
