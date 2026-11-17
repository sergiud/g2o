// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2015-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_ODOMETRY_MEASUREMENT_H
#define G2O_ODOMETRY_MEASUREMENT_H

#include <g2o/core/eigen_types.h>
#include <g2o/prefix.h>
#include <g2o/types/sclam2d/g2o_types_sclam2d_api.h>
#include <g2o/types/sclam2d/odometry_measurement.h>

G2O_START_NAMESPACE

/**
 * \brief velocity measurement of a differential robot
 */
class G2O_TYPES_SCLAM2D_API VelocityMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VelocityMeasurement();
  VelocityMeasurement(double vl, double vr, double dt);

  double vl() const { return _measurement(0); }
  void setVl(double v) { _measurement(0) = v; }

  double vr() const { return _measurement(1); }
  void setVr(double v) { _measurement(1) = v; }

  double dt() const { return _dt; }
  void setDt(double t) { _dt = t; }

  const Vector2D& measurement() const { return _measurement; }

 protected:
  Vector2D _measurement;
  double _dt;
};

/**
 * \brief A 2D odometry measurement expressed as a transformation
 */
class G2O_TYPES_SCLAM2D_API MotionMeasurement {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  MotionMeasurement();
  MotionMeasurement(double x, double y, double theta, double dt);
  MotionMeasurement(const Vector3D& m, double dt);

  double x() const { return _measurement(0); }
  void setX(double v) { _measurement(0) = v; }

  double y() const { return _measurement(1); }
  void setY(double v) { _measurement(1) = v; }

  double theta() const { return _measurement(2); }
  void setTheta(double v) { _measurement(2) = v; }

  double dt() const { return _dt; }
  void setDt(double t) { _dt = t; }

  const Vector3D& measurement() const { return _measurement; }

 protected:
  Vector3D _measurement;
  double _dt;
};

/**
 * \brief convert between the different types of odometry measurements
 */
class G2O_TYPES_SCLAM2D_API OdomConvert {
 public:
  static VelocityMeasurement convertToVelocity(const MotionMeasurement& m);
  static MotionMeasurement convertToMotion(const VelocityMeasurement& vi,
                                           double l = 1.0);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
