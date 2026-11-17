// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_POSE3D_OFFSET_H_
#define G2O_SENSOR_POSE3D_OFFSET_H_
#include "pointsensorparameters.h"
#include "simulator3d_base.h"

G2O_START_NAMESPACE

class SensorPose3DOffset
    : public PointSensorParameters,
      public BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorPose3DOffset(const std::string& name_);
  virtual void sense();
  int stepsToIgnore() const { return _stepsToIgnore; }
  void setStepsToIgnore(int stepsToIgnore_) { _stepsToIgnore = stepsToIgnore_; }
  void addNoise(EdgeType* e);
  virtual void addParameters();
  ParameterSE3Offset* offsetParam1() { return _offsetParam1; };
  ParameterSE3Offset* offsetParam2() { return _offsetParam2; };

 protected:
  bool isVisible(WorldObjectType* to);
  int _stepsToIgnore;
  ParameterSE3Offset *_offsetParam1, *_offsetParam2;

  // these are temporaries
  std::set<PoseObject*> _posesToIgnore;
};

G2O_END_NAMESPACE

#endif
