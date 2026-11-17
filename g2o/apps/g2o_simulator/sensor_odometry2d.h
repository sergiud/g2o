// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_ODOMETRY2D_H_
#define G2O_SENSOR_ODOMETRY2D_H_

#include "sensor_pose2d.h"
#include "sensor_pointxy_bearing.h"

#include <g2o/prefix.h>

G2O_START_NAMESPACE
  class G2O_SIMULATOR_API SensorOdometry2D: public BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2 > {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Robot2D::PoseObject PoseObject;
    SensorOdometry2D(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
