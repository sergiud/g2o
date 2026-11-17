// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include "gtest/gtest.h"

#include "g2o/types/sclam2d/odometry_measurement.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

TEST(Sclam2D, MotionMeasurementCtor)
{
  MotionMeasurement m1(0.1, 0.2, 0.3, 0.5);
  ASSERT_DOUBLE_EQ(0.1, m1.x());
  ASSERT_DOUBLE_EQ(0.2, m1.y());
  ASSERT_DOUBLE_EQ(0.3, m1.theta());
  ASSERT_DOUBLE_EQ(0.5, m1.dt());
  MotionMeasurement m2(Eigen::Vector3d(0.1, 0.2, 0.3), 0.5);
  ASSERT_DOUBLE_EQ(0.1, m2.x());
  ASSERT_DOUBLE_EQ(0.2, m2.y());
  ASSERT_DOUBLE_EQ(0.3, m2.theta());
  ASSERT_DOUBLE_EQ(0.5, m2.dt());
}
TEST(Sclam2D, VelocityMeasurementCtor)
{
  VelocityMeasurement v1(0.1, 0.2, 0.7);
  ASSERT_DOUBLE_EQ(0.1, v1.vl());
  ASSERT_DOUBLE_EQ(0.2, v1.vr());
  ASSERT_DOUBLE_EQ(0.7, v1.dt());
}

TEST(Sclam2D, OdomConvertStraightLine)
{
  MotionMeasurement m1(0.1, 0.0, 0.0, 0.5);
  ASSERT_DOUBLE_EQ(0.1, m1.x());
  ASSERT_DOUBLE_EQ(0.0, m1.y());
  ASSERT_DOUBLE_EQ(0.0, m1.theta());
  ASSERT_DOUBLE_EQ(0.5, m1.dt());
  VelocityMeasurement v1 = OdomConvert::convertToVelocity(m1);
  ASSERT_DOUBLE_EQ(m1.dt(), v1.dt());
  MotionMeasurement m2 = OdomConvert::convertToMotion(v1);
  ASSERT_DOUBLE_EQ(0.1, m2.x());
  ASSERT_DOUBLE_EQ(0.0, m2.y());
  ASSERT_DOUBLE_EQ(0.0, m2.theta());
  ASSERT_DOUBLE_EQ(0.5, m2.dt());
}

TEST(Sclam2D, OdomConvert)
{
  VelocityMeasurement v1(0.1, 0.2, 0.7);
  MotionMeasurement m1 = OdomConvert::convertToMotion(v1);
  VelocityMeasurement v2 = OdomConvert::convertToVelocity(m1);
  ASSERT_DOUBLE_EQ(0.1, v2.vl());
  ASSERT_DOUBLE_EQ(0.2, v2.vr());
  ASSERT_DOUBLE_EQ(0.7, v2.dt());
}
