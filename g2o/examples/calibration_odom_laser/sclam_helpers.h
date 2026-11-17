// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SCLAM_HELPERS_H
#define G2O_SCLAM_HELPERS_H

#include <g2o/core/abi.h>
#include <g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

class SparseOptimizer;
class DataQueue;

G2O_CALIBRATION_ODOM_LASER_API void addOdometryCalibLinksDifferential(
    SparseOptimizer& optimizer, const DataQueue& odomData);

G2O_CALIBRATION_ODOM_LASER_API void allocateSolverForSclam(
    SparseOptimizer& optimizer, bool levenberg = false);

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
