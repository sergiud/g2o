// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_ODOMETRY3D_H_
#define G2O_SENSOR_ODOMETRY3D_H_

#include <g2o/prefix.h>
#include <g2o/simulator/g2o_simulator_api.h>

#include "simulator3d_base.h"

G2O_START_NAMESPACE

class G2O_SIMULATOR_API SensorOdometry3D
    : public BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorOdometry3D(const std::string& name_);
  virtual void sense();
  void addNoise(EdgeType* e);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
