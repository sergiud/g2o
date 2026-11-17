// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_EULER_
#define G2O_EDGE_SE3_EULER_

#include <g2o/prefix.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>

G2O_START_NAMESPACE

/**
 * \brief 3D edge between two VertexSE3, uses the euler angle parameterization
 * for the read/write functions *only*.
 */
class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Euler : public EdgeSE3 {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
