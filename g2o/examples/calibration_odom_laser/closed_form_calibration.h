// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_CLOSED_FORM_CALIBRATION_H
#define G2O_CLOSED_FORM_CALIBRATION_H

#include <g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h>
#include <g2o/examples/calibration_odom_laser/motion_information.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/se2.h>

#include <Eigen/Core>

G2O_START_NAMESPACE

/**
 * \brief Simultaneous calibration of the laser offest and the parameters of a
 * differential drive
 *
 * Approach described by Censi et al.
 * Andrea Censi, Luca Marchionni, and Giuseppe Oriolo.
 * Simultaneous maximum-likelihood calibration of robot and sensor parameters.
 * In Proceedings of the IEEE International Conference on Robotics and
 * Automation (ICRA). Pasadena, CA, May 2008.
 */
class G2O_CALIBRATION_ODOM_LASER_API ClosedFormCalibration {
 public:
  static bool calibrate(const MotionInformationVector& measurements,
                        SE2& laserOffset, Eigen::Vector3d& odomParams);

 protected:
  static Eigen::VectorXd solveLagrange(const Eigen::Matrix<double, 5, 5>& M,
                                       double lambda);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
