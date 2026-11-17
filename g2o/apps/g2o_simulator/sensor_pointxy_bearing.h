// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_POINTXY_BEARING_H_
#define G2O_SENSOR_POINTXY_BEARING_H_

#include <g2o/prefix.h>
#include <g2o/simulator/g2o_simulator_api.h>

#include "pointsensorparameters.h"
#include "simulator2d_base.h"

G2O_START_NAMESPACE

class G2O_SIMULATOR_API SensorPointXYBearing
    : public PointSensorParameters,
      public BinarySensor<Robot2D, EdgeSE2PointXYBearing, WorldObjectPointXY> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorPointXYBearing(const std::string& name_);
  virtual void addNoise(EdgeType* e);
  virtual void sense();

 protected:
  bool isVisible(WorldObjectType* to);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
