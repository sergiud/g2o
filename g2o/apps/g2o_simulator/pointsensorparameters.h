// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_POINTSENSORPARAMETERS_H_
#define G2O_POINTSENSORPARAMETERS_H_

#include <g2o/core/abi.h>
#include <g2o/prefix.h>
#include <g2o/simulator/g2o_simulator_api.h>

#include <cmath>

G2O_START_NAMESPACE

class G2O_SIMULATOR_API PointSensorParameters {
 public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PointSensorParameters();
  double maxRange() const { return sqrt(_maxRange2); }
  void setMaxRange(double maxRange_) { _maxRange2 = maxRange_ * maxRange_; }
  double minRange() const { return sqrt(_minRange2); }
  void setMinRange(double minRange_) { _minRange2 = minRange_ * minRange_; }
  double fov() const { return _fov; }
  void setFov(double fov_) { _fov = fov_; }
  double maxAngularDifference() const { return _maxAngularDifference; }
  void setMaxAngularDifference(double angularDifference) {
    _maxAngularDifference = angularDifference;
  }

 protected:
  double _maxRange2, _minRange2, _fov, _maxAngularDifference;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
