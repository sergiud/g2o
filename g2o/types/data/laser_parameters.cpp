// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/data/laser_parameters.h>

G2O_START_NAMESPACE

  LaserParameters::LaserParameters(int t, int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange, double _accuracy, int _remissionMode)
  {
    type           = t;
    firstBeamAngle = _firstBeamAngle;
    angularStep    = _angularStep;
    maxRange       = _maxRange;
    laserPose      = SE2(0., 0., 0.);
    accuracy       = _accuracy;
    remissionMode  = _remissionMode;
    fov            = angularStep * nbeams;
  }

  LaserParameters::LaserParameters(int nbeams, double _firstBeamAngle, double _angularStep, double _maxRange)
  {
    type           = 0;
    firstBeamAngle = _firstBeamAngle;
    angularStep    = _angularStep;
    maxRange       = _maxRange;
    laserPose      = SE2(0., 0., 0.);
    accuracy       = 0.1;
    remissionMode  = 0;
    fov            = angularStep * nbeams;
  }

G2O_END_NAMESPACE
