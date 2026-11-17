// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/creators.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/types/data/robot_laser.h>
#include <g2o/types/data/vertex_ellipse.h>
#include <g2o/types/data/vertex_tag.h>

G2O_START_NAMESPACE

G2O_REGISTER_TYPE_GROUP(data);

G2O_REGISTER_TYPE(VERTEX_TAG, VertexTag);
G2O_REGISTER_TYPE(ROBOTLASER1, RobotLaser);
G2O_REGISTER_TYPE(VERTEX_ELLIPSE, VertexEllipse);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(RobotLaserDrawAction);
G2O_REGISTER_ACTION(VertexTagDrawAction);
G2O_REGISTER_ACTION(VertexEllipseDrawAction);
#endif

G2O_END_NAMESPACE
