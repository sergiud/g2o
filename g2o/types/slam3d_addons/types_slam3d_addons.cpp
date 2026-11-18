// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <iostream>
#include <memory>
#include <typeinfo>
#include <utility>

G2O_START_NAMESPACE

G2O_REGISTER_TYPE_GROUP(slam3d_addons);

G2O_REGISTER_TYPE(VERTEX3, VertexSE3Euler);
G2O_REGISTER_TYPE(EDGE3, EdgeSE3Euler);
G2O_REGISTER_TYPE(VERTEX_PLANE, VertexPlane);
G2O_REGISTER_TYPE(EDGE_SE3_PLANE_CALIB, EdgeSE3PlaneSensorCalib);

G2O_REGISTER_TYPE(VERTEX_LINE3D, VertexLine3D);
G2O_REGISTER_TYPE(EDGE_SE3_LINE3D, EdgeSE3Line3D);
G2O_REGISTER_TYPE(EDGE_PLANE, EdgePlane);
G2O_REGISTER_TYPE(EDGE_SE3_CALIB, EdgeSE3Calib);

#ifdef G2O_HAVE_OPENGL
G2O_REGISTER_ACTION(CacheCameraDrawAction);
G2O_REGISTER_ACTION(VertexPlaneDrawAction);
G2O_REGISTER_ACTION(EdgeSE3PlaneSensorCalibDrawAction);
G2O_REGISTER_ACTION(VertexLine3DDrawAction);
G2O_REGISTER_ACTION(EdgeSE3Line3DDrawAction);
#endif

G2O_ATTRIBUTE_CONSTRUCTOR(init_slam3d_addons_types) {
  static bool initialized = false;
  if (initialized) return;
  initialized = true;

#ifdef G2O_HAVE_OPENGL
  HyperGraphActionLibrary* actionLib = HyperGraphActionLibrary::instance();
  auto vertexse3eulerdraw = std::make_unique<g2o::VertexSE3DrawAction>();
  vertexse3eulerdraw->setTypeName(typeid(VertexSE3Euler).name());
  actionLib->registerAction(std::move(vertexse3eulerdraw));

  auto edgese3eulerdraw = std::make_unique<g2o::EdgeSE3DrawAction>();
  edgese3eulerdraw->setTypeName(typeid(EdgeSE3Euler).name());
  actionLib->registerAction(std::move(edgese3eulerdraw));
#endif
}

G2O_END_NAMESPACE
