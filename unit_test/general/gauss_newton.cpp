// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

/**
 * A system that the linear solver cannot factorize has no solution, so the
 * step that follows would apply what the solution vector held before.
 */
TEST(GaussNewton, KeepsTheEstimateWhenTheSolverFails) {
  using Block = g2o::BlockSolverX;
  constexpr int Poses = 3;
  g2o::SparseOptimizer graph;
  graph.setAlgorithm(std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(
      std::make_unique<Block>(
          std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>())));
  for (int i = 0; i < Poses; ++i) {
    auto pose = std::make_unique<g2o::VertexSE2>();
    pose->setId(i);
    pose->setEstimate(g2o::SE2(i, 0., 0.));
    pose->setFixed(i == 0);
    ASSERT_TRUE(graph.addVertex(std::move(pose)));
  }
  for (int i = 0; i + 1 < Poses; ++i) {
    auto edge = std::make_unique<g2o::EdgeSE2>();
    edge->setVertex(0, graph.vertex(i));
    edge->setVertex(1, graph.vertex(i + 1));
    edge->setMeasurement(g2o::SE2(1., 0.1, 0.05));
    // Without information the Hessian is zero and has no factorization.
    edge->setInformation(Eigen::Matrix3d::Zero());
    ASSERT_TRUE(graph.addEdge(std::move(edge)));
  }
  ASSERT_TRUE(graph.initializeOptimization());

  EXPECT_EQ(graph.optimize(1), 0);

  for (int i = 0; i < Poses; ++i) {
    const auto* pose = static_cast<const g2o::VertexSE2*>(graph.vertex(i));
    EXPECT_EQ(pose->estimate().toVector(), g2o::SE2(i, 0., 0.).toVector())
        << "pose " << i;
  }
}

//! An algorithm writes debug files of failed solves only on request.
TEST(GaussNewton, WritesNoDebugFilesByDefault) {
  using Block = g2o::BlockSolverX;
  const g2o::OptimizationAlgorithmGaussNewton algorithm(std::make_unique<Block>(
      std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>()));

  EXPECT_FALSE(algorithm.writeDebug());
}
