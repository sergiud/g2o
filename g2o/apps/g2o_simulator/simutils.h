// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SIMUTILS_H_
#define G2O_SIMUTILS_H_

#include <g2o/prefix.h>
#include <g2o/simulator/g2o_simulator_api.h>

#include "pointsensorparameters.h"
#include "simulator2d_base.h"

G2O_START_NAMESPACE
// -1: outside
// 0: p1Clipped
// 1: p2clipped
// 2: inside
// 3: all clipped

G2O_SIMULATOR_API int clipSegmentCircle(Eigen::Vector2d& p1,
                                        Eigen::Vector2d& p2, double r);

// -1: outside
// 0: p1Clipped
// 1: p2clipped
// 2: inside
G2O_SIMULATOR_API int clipSegmentLine(Eigen::Vector2d& p1, Eigen::Vector2d& p2,
                                      double a, double b, double c);

// -1: outside
// 0: p1Clipped
// 1: p2clipped
// 2: inside
// 3: all clipped
G2O_SIMULATOR_API int clipSegmentFov(Eigen::Vector2d& p1, Eigen::Vector2d& p2,
                                     double min, double max);

G2O_SIMULATOR_API Eigen::Vector2d computeLineParameters(
    const Eigen::Vector2d& p1, const Eigen::Vector2d& p2);
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
