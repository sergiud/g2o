// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_MOTION_INFORMATION_H
#define G2O_MOTION_INFORMATION_H

#include <g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h>
#include <g2o/prefix.h>
#include <g2o/types/slam2d/se2.h>

#include <Eigen/StdVector>
#include <vector>

G2O_START_NAMESPACE

struct G2O_CALIBRATION_ODOM_LASER_API MotionInformation {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  SE2 laserMotion;
  SE2 odomMotion;
  double timeInterval{};
};

typedef std::vector<MotionInformation,
                    Eigen::aligned_allocator<MotionInformation> >
    MotionInformationVector;

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
