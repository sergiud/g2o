// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>

#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_property.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

G2O_USE_OPTIMIZATION_LIBRARY(slam2d_linear)

using ::testing::DistanceFrom;
using ::testing::Le;

/**
 * The linear solver computes the orientations relative to the fixed vertex,
 * which keeps its pose rather than moving to the origin.
 */
TEST(SolverSLAM2DLinear, KeepsTheFixedVertex) {
  constexpr int Poses = 3;
  // The orientation step makes the translations linear, so a single
  // iteration solves them only after correct orientations.
  constexpr int Iterations = 1;
  constexpr double Tolerance = 1e-9;
  const g2o::SE2 fixed(5., 5., 0.5);
  const g2o::SE2 odometry(1., 0., 0.1);
  g2o::SparseOptimizer graph;
  g2o::OptimizationAlgorithmProperty property;
  graph.setAlgorithm(g2o::OptimizationAlgorithmFactory::instance()->construct(
      "2dlinear", property));
  ASSERT_NE(graph.algorithm(), nullptr);
  g2o::SE2 pose = fixed;
  for (int i = 0; i < Poses; ++i) {
    auto vertex = std::make_unique<g2o::VertexSE2>();
    vertex->setId(i);
    // The free poses start away from the poses the odometry implies.
    vertex->setEstimate(i == 0 ? pose : g2o::SE2(i, -i, 0.));
    vertex->setFixed(i == 0);
    ASSERT_TRUE(graph.addVertex(std::move(vertex)));
    pose *= odometry;
  }
  for (int i = 0; i + 1 < Poses; ++i) {
    auto edge = std::make_unique<g2o::EdgeSE2>();
    edge->setVertex(0, graph.vertex(i));
    edge->setVertex(1, graph.vertex(i + 1));
    edge->setMeasurement(odometry);
    edge->setInformation(Eigen::Matrix3d::Identity());
    ASSERT_TRUE(graph.addEdge(std::move(edge)));
  }
  // The loop closes at the fixed vertex, which the edge reaches last.
  auto closure = std::make_unique<g2o::EdgeSE2>();
  closure->setVertex(0, graph.vertex(Poses - 1));
  closure->setVertex(1, graph.vertex(0));
  closure->setMeasurement((odometry * odometry).inverse());
  closure->setInformation(Eigen::Matrix3d::Identity());
  ASSERT_TRUE(graph.addEdge(std::move(closure)));
  ASSERT_TRUE(graph.initializeOptimization());

  EXPECT_GT(graph.optimize(Iterations), 0);

  g2o::SE2 expected = fixed;
  for (int i = 0; i < Poses; ++i) {
    const auto* vertex = static_cast<const g2o::VertexSE2*>(graph.vertex(i));
    EXPECT_THAT(vertex->estimate().toVector(),
                DistanceFrom(expected.toVector(), g2o::test::MaxAbsDifference,
                             Le(Tolerance)))
        << "pose " << i;
    expected *= odometry;
  }
}
