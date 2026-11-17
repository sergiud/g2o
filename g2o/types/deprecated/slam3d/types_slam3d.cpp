// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "types_slam3d.h"

#include <g2o/core/factory.h>

G2O_START_NAMESPACE
namespace deprecated {

G2O_REGISTER_TYPE_GROUP(deprecated_slam3d);

// cannot register the types right now, as the macros use the class name
// which declares duplicate functions, if also linking with the default
// slam3d types
#if 0
  G2O_REGISTER_TYPE_NAME("DEPRECATED_VERTEX_SE3:QUAT", VertexSE3);
  G2O_REGISTER_TYPE_NAME("DEPRECATED_EDGE_SE3:QUAT", EdgeSE3);
  G2O_REGISTER_TYPE(DEPRECATED_VERTEX_TRACKXYZ, VertexPointXYZ);

  G2O_REGISTER_TYPE(DEPRECATED_PARAMS_SE3OFFSET, ParameterSE3Offset);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_TRACKXYZ, EdgeSE3PointXYZ);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_PRIOR, EdgeSE3Prior);
  G2O_REGISTER_TYPE(DEPRECATED_CACHE_SE3_OFFSET, CacheSE3Offset);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_SE3_OFFSET, EdgeSE3Offset);

  G2O_REGISTER_TYPE(DEPRECATED_PARAMS_CAMERACALIB, ParameterCamera);
  G2O_REGISTER_TYPE(DEPRECATED_CACHE_CAMERA, CacheCamera);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_PROJECT_DISPARITY, EdgeSE3PointXYZDisparity);
  G2O_REGISTER_TYPE(DEPRECATED_EDGE_PROJECT_DEPTH, EdgeSE3PointXYZDepth);

  /*********** ACTIONS ************/
  G2O_REGISTER_ACTION(VertexSE3WriteGnuplotAction);
  G2O_REGISTER_ACTION(VertexPointXYZWriteGnuplotAction);
  G2O_REGISTER_ACTION(EdgeSE3WriteGnuplotAction);

#ifdef G2O_HAVE_OPENGL
  G2O_REGISTER_ACTION(VertexPointXYZDrawAction);
  G2O_REGISTER_ACTION(VertexSE3DrawAction);
  G2O_REGISTER_ACTION(EdgeSE3DrawAction);
  G2O_REGISTER_ACTION(CacheCameraDrawAction);
#endif
#endif

}  // end namespace
}  // end namespace
