// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>

#include <iostream>

G2O_START_NAMESPACE

G2O_REGISTER_TYPE_GROUP(slam2d_segment);

G2O_REGISTER_TYPE(VERTEX_SEGMENT2D, VertexSegment2D);
G2O_REGISTER_TYPE(VERTEX_LINE2D, VertexLine2D);

G2O_REGISTER_TYPE(EDGE_SE2_SEGMENT2D, EdgeSE2Segment2D);
G2O_REGISTER_TYPE(EDGE_SE2_SEGMENT2D_LINE, EdgeSE2Segment2DLine);
G2O_REGISTER_TYPE(EDGE_SE2_SEGMENT2D_POINTLINE, EdgeSE2Segment2DPointLine);
G2O_REGISTER_TYPE(EDGE_SE2_LINE2D, EdgeSE2Line2D);
G2O_REGISTER_TYPE(EDGE_LINE2D, EdgeLine2D);
G2O_REGISTER_TYPE(EDGE_LINE2D_POINTXY, EdgeLine2DPointXY);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(VertexSegment2DDrawAction);
G2O_REGISTER_ACTION(VertexLine2DDrawAction);
#endif

G2O_END_NAMESPACE
