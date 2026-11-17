// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "pointsensorparameters.h"

#include <cmath>

// Robot2D
G2O_START_NAMESPACE
  PointSensorParameters::PointSensorParameters(){
    _maxRange2 = 25;
    _minRange2 = 0.01;
    _fov = M_PI/2;
    _maxAngularDifference = M_PI/2;
  }
G2O_END_NAMESPACE
