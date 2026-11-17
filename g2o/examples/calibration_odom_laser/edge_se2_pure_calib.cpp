// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "edge_se2_pure_calib.h"

G2O_START_NAMESPACE

EdgeSE2PureCalib::EdgeSE2PureCalib() {}

bool EdgeSE2PureCalib::read(std::istream& is) {
  (void)is;
  return false;
}

bool EdgeSE2PureCalib::write(std::ostream& os) const {
  (void)os;
  return false;
}

void EdgeSE2PureCalib::computeError() {
  const VertexSE2* laserOffset = dynamic_cast<const VertexSE2*>(_vertices[0]);
  const VertexOdomDifferentialParams* odomParams =
      dynamic_cast<const VertexOdomDifferentialParams*>(_vertices[1]);

  // get the calibrated motion given by the odometry
  VelocityMeasurement calibratedVelocityMeasurment(
      measurement().velocityMeasurement.vl() * odomParams->estimate()(0),
      measurement().velocityMeasurement.vr() * odomParams->estimate()(1),
      measurement().velocityMeasurement.dt());
  MotionMeasurement mm = OdomConvert::convertToMotion(
      calibratedVelocityMeasurment, odomParams->estimate()(2));
  SE2 Ku_ij;
  Ku_ij.fromVector(mm.measurement());

  SE2 laserMotionInRobotFrame = laserOffset->estimate() *
                                measurement().laserMotion *
                                laserOffset->estimate().inverse();
  SE2 delta = Ku_ij.inverse() * laserMotionInRobotFrame;
  _error = delta.toVector();
}

G2O_END_NAMESPACE
