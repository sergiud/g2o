// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/sclam2d/odometry_measurement.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

G2O_START_NAMESPACE

VelocityMeasurement::VelocityMeasurement() : _measurement(0., 0.), _dt(0.) {}

VelocityMeasurement::VelocityMeasurement(double vl, double vr, double dt)
    : _measurement(vl, vr), _dt(dt) {}

MotionMeasurement::MotionMeasurement() : _measurement(0., 0., 0.), _dt(0.) {}

MotionMeasurement::MotionMeasurement(double x, double y, double theta,
                                     double dt)
    : _measurement(x, y, theta), _dt(dt) {}

MotionMeasurement::MotionMeasurement(const Vector3D& m, double dt)
    : _measurement(m), _dt(dt) {}

VelocityMeasurement OdomConvert::convertToVelocity(const MotionMeasurement& m) {
  Vector2D px2(0, 10);

  if (fabs(m.theta()) > 1e-7) {
    Eigen::Rotation2Dd rot(m.theta());
    Vector2D px3(m.x(), m.y());
    Vector2D px4(rot * px2 + px3);

    const double& y2 = px2.y();
    const double& x3 = px3.x();
    const double& y3 = px3.y();
    const double& x4 = px4.x();
    const double& y4 = px4.y();

    double R = (y2 * (x3 * y4 - y3 * x4)) / (y2 * (x3 - x4));
    double w;
    if (fabs(m.dt()) > 1e-7)
      w = m.theta() / m.dt();
    else
      w = 0.;

    double vl = (2. * R * w - w) / 2.;
    double vr = w + vl;

    return VelocityMeasurement(vl, vr, m.dt());
  } else {
    double vl, vr;
    // Without rotation, a differential drive moves along its heading, whose
    // direction the sign of the forward motion keeps.
    if (fabs(m.dt()) > 1e-7)
      vl = vr = m.x() / m.dt();
    else
      vl = vr = 0.;
    return VelocityMeasurement(vl, vr, m.dt());
  }
}

MotionMeasurement OdomConvert::convertToMotion(const VelocityMeasurement& v,
                                               double l) {
  double x, y, theta;
  if (fabs(v.vr() - v.vl()) > 1e-7) {
    double R = l * 0.5 * ((v.vl() + v.vr()) / (v.vr() - v.vl()));
    double w = (v.vr() - v.vl()) / l;

    theta = w * v.dt();
    Eigen::Rotation2Dd rot(theta);
    Vector2D icc(0, R);
    Vector2D motion = (rot * (Vector2D(-1. * icc))) + icc;
    x = motion.x();
    y = motion.y();
  } else {
    double tv = 0.5 * (v.vr() + v.vl());
    theta = 0.;
    x = tv * v.dt();
    y = 0.;
  }

  return MotionMeasurement(x, y, theta, v.dt());
}

G2O_END_NAMESPACE
