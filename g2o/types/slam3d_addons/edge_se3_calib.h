// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_CALIB_H
#define G2O_EDGE_SE3_CALIB_H

#include "Eigen/Geometry"
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE
  /**
   * \brief Landmark measurement that also calibrates an offset for the landmark measurement
   */
  class G2O_TYPES_SLAM3D_ADDONS_API EdgeSE3Calib : public BaseMultiEdge<6, Isometry3D> //Avoid redefinition of BaseEdge in MSVC
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE3Calib();

      void computeError();
      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
