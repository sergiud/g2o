// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam2d/edge_se2_pointxy_calib.h"

#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

/**
 * The landmark lies where the calibrated sensor of the robot measures it,
 * which leaves the edge without an error.
 */
TEST(EdgeSE2PointXYCalib, InitializesTheLandmarkThroughTheCalibration) {
  constexpr double Tolerance = 1e-12;
  g2o::VertexSE2 robot;
  robot.setEstimate(g2o::SE2(1., -2., 0.4));
  g2o::VertexPointXY landmark;
  g2o::VertexSE2 calibration;
  calibration.setEstimate(g2o::SE2(1., 0., M_PI / 2.));
  g2o::EdgeSE2PointXYCalib edge;
  edge.setVertex(0, &robot);
  edge.setVertex(1, &landmark);
  edge.setVertex(2, &calibration);
  edge.setMeasurement(Eigen::Vector2d(2., 0.));
  g2o::OptimizableGraph::VertexSet from{&robot};

  edge.initialEstimate(from, &landmark);
  edge.computeError();

  EXPECT_THAT(edge.error(),
              DistanceFrom(Eigen::Vector2d::Zero(), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}
