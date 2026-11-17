// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/sclam2d/edge_se2_odom_differential_calib.h>
#include <g2o/types/sclam2d/edge_se2_sensor_calib.h>
#include <g2o/types/slam2d/types_slam2d.h>

G2O_START_NAMESPACE

G2O_USE_TYPE_GROUP(slam2d);

G2O_REGISTER_TYPE_GROUP(sclam);
G2O_REGISTER_TYPE(VERTEX_ODOM_DIFFERENTIAL, VertexOdomDifferentialParams);
G2O_REGISTER_TYPE(EDGE_SE2_CALIB, EdgeSE2SensorCalib);
G2O_REGISTER_TYPE(EDGE_SE2_ODOM_DIFFERENTIAL_CALIB,
                  EdgeSE2OdomDifferentialCalib);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(EdgeSE2SensorCalibDrawAction);
G2O_REGISTER_ACTION(EdgeSE2OdomDifferentialCalibDrawAction);
#endif

G2O_END_NAMESPACE
