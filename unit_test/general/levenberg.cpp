// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <vector>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;
using ::testing::Pointwise;

namespace {

/**
 * Poses that Levenberg-Marquardt optimizes from a distorted chain, whose
 * information is scaled by the given factor.
 */
std::vector<double> optimizedPoses(double informationScale) {
  using Block = g2o::BlockSolverX;
  constexpr int Poses = 20;
  constexpr int Iterations = 4;
  g2o::SparseOptimizer graph;
  graph.setAlgorithm(std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
      std::make_unique<Block>(
          std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>())));
  for (int i = 0; i < Poses; ++i) {
    auto pose = std::make_unique<g2o::VertexSE2>();
    pose->setId(i);
    pose->setEstimate(g2o::SE2(i, 0.5 * (i % 3), 0.3 * (i % 2)));
    pose->setFixed(i == 0);
    graph.addVertex(std::move(pose));
  }
  for (int i = 0; i + 1 < Poses; ++i) {
    auto edge = std::make_unique<g2o::EdgeSE2>();
    edge->setVertex(0, graph.vertex(i));
    edge->setVertex(1, graph.vertex(i + 1));
    edge->setMeasurement(g2o::SE2(1., 0.1, 0.05));
    edge->setInformation(informationScale * Eigen::Matrix3d::Identity());
    graph.addEdge(std::move(edge));
  }
  graph.initializeOptimization();
  graph.optimize(Iterations);
  std::vector<double> poses;
  for (int i = 0; i < Poses; ++i) {
    const Eigen::Vector3d pose =
        static_cast<const g2o::VertexSE2*>(graph.vertex(i))
            ->estimate()
            .toVector();
    poses.insert(poses.end(), pose.begin(), pose.end());
  }
  return poses;
}

}  // namespace

/**
 * Scaling every information by the same factor scales the cost but not its
 * minimum, so Levenberg-Marquardt takes the same steps.
 */
TEST(Levenberg, IgnoresTheScaleOfTheInformation) {
  constexpr double SmallScale = 1e-8;
  constexpr double Tolerance = 1e-9;

  EXPECT_THAT(optimizedPoses(SmallScale),
              Pointwise(DoubleNear(Tolerance), optimizedPoses(1.)));
}
