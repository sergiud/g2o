// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LASER_PARAMETERS_H
#define G2O_LASER_PARAMETERS_H

#include <g2o/prefix.h>
#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/types/slam2d/se2.h>

G2O_START_NAMESPACE

/**
 * \brief parameters for a 2D range finder
 */
struct G2O_TYPES_DATA_API LaserParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  LaserParameters(int type, int beams, double firstBeamAngle,
                  double angularStep, double maxRange, double accuracy,
                  int remissionMode);
  LaserParameters(int beams, double firstBeamAngle, double angularStep,
                  double maxRange);
  SE2 laserPose;
  int type;
  double firstBeamAngle;
  double fov;
  double angularStep;
  double accuracy;
  int remissionMode;
  double maxRange;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
