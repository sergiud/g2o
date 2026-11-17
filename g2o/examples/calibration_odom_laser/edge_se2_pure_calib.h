// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef EDGE_SE2_PURE_CALIB_H
#define EDGE_SE2_PURE_CALIB_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h>
#include <g2o/types/sclam2d/odometry_measurement.h>
#include <g2o/types/sclam2d/vertex_odom_differential_params.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  struct G2O_CALIBRATION_ODOM_LASER_API OdomAndLaserMotion
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VelocityMeasurement velocityMeasurement;
    SE2 laserMotion;
  };

#ifndef calibration_odom_laser_library_EXPORTS
  G2O_EXTERN_TEMPLATE
#else
  template
#endif
  class G2O_CALIBRATION_ODOM_LASER_API BaseEdge<3, OdomAndLaserMotion>;

  /**
   * \brief calibrate odometry and laser based on a set of measurements
   */
  class G2O_CALIBRATION_ODOM_LASER_API EdgeSE2PureCalib : public BaseBinaryEdge<3, OdomAndLaserMotion, VertexSE2, VertexOdomDifferentialParams>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      EdgeSE2PureCalib();

      void computeError();

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
