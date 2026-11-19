// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam2d/edge_se2_pointxy_offset.h"

#include <memory>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"
#include "unit_test/test_helper/evaluate_jacobian.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

//! A robot that observes a landmark through a sensor with an offset.
class PointXYOffset : public ::testing::Test {
 protected:
  void SetUp() override {
    auto offset = std::make_unique<g2o::ParameterSE2Offset>();
    offset->setId(0);
    _offset = offset.get();
    ASSERT_TRUE(_graph.addParameter(std::move(offset)));
    auto robot = std::make_unique<g2o::VertexSE2>();
    robot->setId(0);
    _robot = robot.get();
    auto landmark = std::make_unique<g2o::VertexPointXY>();
    landmark->setId(1);
    _landmark = landmark.get();
    ASSERT_TRUE(_graph.addVertex(std::move(robot)));
    ASSERT_TRUE(_graph.addVertex(std::move(landmark)));
    auto edge = std::make_unique<g2o::EdgeSE2PointXYOffset>();
    _edge = edge.get();
    _edge->setVertex(0, _robot);
    _edge->setVertex(1, _landmark);
    ASSERT_TRUE(_edge->setParameterId(0, 0));
    ASSERT_TRUE(_graph.addEdge(std::move(edge)));
  }

  //! Places the robot, the sensor on it and the landmark.
  void place(const g2o::SE2& robot, const g2o::SE2& offset,
             const Eigen::Vector2d& landmark) {
    _offset->setOffset(offset);
    _robot->setEstimate(robot);
    _landmark->setEstimate(landmark);
  }

  g2o::SparseOptimizer _graph;
  g2o::ParameterSE2Offset* _offset = nullptr;
  g2o::VertexSE2* _robot = nullptr;
  g2o::VertexPointXY* _landmark = nullptr;
  g2o::EdgeSE2PointXYOffset* _edge = nullptr;
};

constexpr double Tolerance = 1e-12;

}  // namespace

/**
 * The edge measures the landmark in the frame of the sensor, so the position
 * of the landmark there leaves no error.
 */
TEST_F(PointXYOffset, MeasuresInTheSensorFrame) {
  const g2o::SE2 robot(1., 2., 0.3);
  const g2o::SE2 offset(0.5, -0.2, 0.7);
  const Eigen::Vector2d landmark(4., -1.);
  place(robot, offset, landmark);
  _edge->setMeasurement((robot * offset).inverse() * landmark);

  _edge->computeError();

  EXPECT_THAT(_edge->error(),
              DistanceFrom(Eigen::Vector2d::Zero(), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

//! The rotation of the sensor enters every block of the Jacobian.
TEST_F(PointXYOffset, Jacobian) {
  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(_edge);
  numericJacobianWorkspace.allocate();

  constexpr int Samples = 1000;
  for (int k = 0; k < Samples; ++k) {
    place(g2o::SE2(Eigen::Vector3d::Random()),
          g2o::SE2(Eigen::Vector3d::Random()), Eigen::Vector2d::Random());
    _edge->setMeasurement(Eigen::Vector2d::Random());

    evaluateJacobian(*_edge, jacobianWorkspace, numericJacobianWorkspace);
  }
}

//! The measurement is a position in the plane.
TEST_F(PointXYOffset, MeasurementHasTwoDimensions) {
  EXPECT_EQ(_edge->measurementDimension(), 2);
}

//! The measurement of the current state is the landmark seen by the sensor.
TEST_F(PointXYOffset, MeasuresTheStateInTheSensorFrame) {
  const g2o::SE2 robot(1., 2., 0.3);
  const g2o::SE2 offset(0.5, -0.2, 0.7);
  const Eigen::Vector2d landmark(4., -1.);
  place(robot, offset, landmark);

  ASSERT_TRUE(_edge->setMeasurementFromState());

  EXPECT_THAT(
      _edge->measurement(),
      DistanceFrom(Eigen::Vector2d((robot * offset).inverse() * landmark),
                   g2o::test::MaxAbsDifference, Le(Tolerance)));
}
