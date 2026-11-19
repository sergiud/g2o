// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::Le;

/**
 * A graph whose measurements agree with its estimates has nothing left to
 * gain, which ends the optimization after its first iterations instead of
 * running all of them.
 */
TEST(SparseOptimizerTerminateAction, StopsAtAnExactFit) {
  constexpr int MaxIterations = 100;
  constexpr int Converged = 3;
  using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  g2o::SparseOptimizerTerminateAction terminate;
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(
      std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(
          std::make_unique<BlockSolver>(
              std::make_unique<
                  g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>>())));
  optimizer.addPostIterationAction(&terminate);
  Eigen::Isometry3d step = Eigen::Isometry3d::Identity();
  step.translation() = Eigen::Vector3d(1., 0., 0.);
  auto first = std::make_unique<g2o::VertexSE3>();
  first->setId(0);
  first->setFixed(true);
  auto second = std::make_unique<g2o::VertexSE3>();
  second->setId(1);
  second->setEstimate(step);
  auto edge = std::make_unique<g2o::EdgeSE3>();
  edge->setVertex(0, first.get());
  edge->setVertex(1, second.get());
  edge->setMeasurement(step);
  edge->setInformation(g2o::EdgeSE3::InformationType::Identity());
  ASSERT_TRUE(optimizer.addVertex(std::move(first)));
  ASSERT_TRUE(optimizer.addVertex(std::move(second)));
  ASSERT_TRUE(optimizer.addEdge(std::move(edge)));
  ASSERT_TRUE(optimizer.initializeOptimization());

  EXPECT_THAT(optimizer.optimize(MaxIterations), Le(Converged));
}
