// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_POSE3D_H_
#define G2O_SENSOR_POSE3D_H_
#include <g2o/prefix.h>

#include "pointsensorparameters.h"
#include "sensor_pointxyz.h"
#include "simulator3d_base.h"

G2O_START_NAMESPACE

class G2O_SIMULATOR_API SensorPose3D
    : public PointSensorParameters,
      public BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SensorPose3D(const std::string& name_);
  virtual void sense();
  int stepsToIgnore() const { return _stepsToIgnore; }
  void setStepsToIgnore(int stepsToIgnore_) { _stepsToIgnore = stepsToIgnore_; }
  void addNoise(EdgeType* e);

 protected:
  bool isVisible(WorldObjectType* to);
  int _stepsToIgnore;
  // these are temporaries
  std::set<PoseObject*> _posesToIgnore;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
