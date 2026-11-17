// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_LINE3D_H_
#define G2O_SENSOR_LINE3D_H_
#include <g2o/prefix.h>
#include <g2o/simulator/g2o_simulator_api.h>

#include "pointsensorparameters.h"
#include "simulator3d_base.h"

G2O_START_NAMESPACE

class G2O_SIMULATOR_API SensorPointLine3D
    : public PointSensorParameters,
      public BinarySensor<Robot3D, EdgeSE3Line WorldObjectLine3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef PoseVertexType::EstimateType RobotPoseType;
  SensorLine3D(const std::string& name_);
  virtual void sense();
  virtual void addParameters();
  ParameterSE3Offset* offsetParam() { return _offsetParam; };
  void addNoise(EdgeType* e);

 protected:
  bool isVisible(WorldObjectType* to);
  RobotPoseType _sensorPose;
  ParameterSE3Offset* _offsetParam;
};
}

#endif
