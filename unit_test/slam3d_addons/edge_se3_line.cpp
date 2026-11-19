// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d_addons/edge_se3_line.h"

#include <memory>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d_addons/vertex_line3d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

constexpr double Tolerance = 1e-9;

//! A robot that observes a line through a sensor with an offset.
class LineThroughOffset : public ::testing::Test {
 protected:
  void SetUp() override {
    g2o::Vector6d cartesian;
    cartesian << 3., 1., 2., 0.6, 0., 0.8;
    _line = g2o::Line3D::fromCartesian(cartesian);
    auto parameter = std::make_unique<g2o::ParameterSE3Offset>();
    parameter->setId(0);
    parameter->setOffset(_offset);
    ASSERT_TRUE(_graph.addParameter(std::move(parameter)));
    auto pose = std::make_unique<g2o::VertexSE3>();
    pose->setId(0);
    pose->setEstimate(_robot);
    auto landmark = std::make_unique<g2o::VertexLine3D>();
    landmark->setId(1);
    landmark->setEstimate(_line);
    auto edge = std::make_unique<g2o::EdgeSE3Line3D>();
    _edge = edge.get();
    edge->setVertex(0, pose.get());
    edge->setVertex(1, landmark.get());
    ASSERT_TRUE(edge->setParameterId(0, 0));
    ASSERT_TRUE(_graph.addVertex(std::move(pose)));
    ASSERT_TRUE(_graph.addVertex(std::move(landmark)));
    ASSERT_TRUE(_graph.addEdge(std::move(edge)));
  }

  //! The line as the sensor sees it.
  g2o::Line3D lineInSensorFrame() const {
    return (_robot * _offset).inverse() * _line;
  }

  const Eigen::Isometry3d _robot =
      Eigen::Translation3d(1., 2., 0.5) *
      Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ());
  const Eigen::Isometry3d _offset =
      Eigen::Translation3d(0.2, -0.1, 0.4) *
      Eigen::AngleAxisd(-0.8, Eigen::Vector3d::UnitY());
  g2o::Line3D _line;
  g2o::SparseOptimizer _graph;
  g2o::EdgeSE3Line3D* _edge = nullptr;
};

}  // namespace

/**
 * A sensor with an offset observes a line in its own frame, so the line as
 * the sensor sees it leaves no error.
 */
TEST_F(LineThroughOffset, MeasuresInTheSensorFrame) {
  _edge->setMeasurement(lineInSensorFrame());

  _edge->computeError();

  EXPECT_THAT(_edge->error(),
              DistanceFrom(Eigen::Vector4d::Zero(), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

//! A simulated sensor measures the current state, as the sensor sees it.
TEST_F(LineThroughOffset, MeasuresTheState) {
  ASSERT_TRUE(_edge->setMeasurementFromState());

  EXPECT_THAT(_edge->measurement(),
              DistanceFrom(lineInSensorFrame(), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}
