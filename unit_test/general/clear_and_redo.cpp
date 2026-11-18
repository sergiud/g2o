// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/sparse_optimizer_terminate_action.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gtest/gtest.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>
    SlamLinearSolver;

TEST(General, ClearAndRedo) {
  // The optimizer uses the terminate action, which outlives it.
  g2o::SparseOptimizerTerminateAction terminateAction;
  // Initialize the SparseOptimizer
  g2o::SparseOptimizer mOptimizer;
  auto linearSolver = std::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
  mOptimizer.setAlgorithm(
      std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(
          std::move(blockSolver)));

  // Set the default terminate action
  mOptimizer.addPostIterationAction(&terminateAction);

  for (int i = 0; i < 2; i++) {
    // Add vertices
    auto v0Owner = std::make_unique<g2o::VertexSE3>();
    g2o::VertexSE3* v0 = v0Owner.get();
    v0->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v0->setId(0);
    ASSERT_TRUE(mOptimizer.addVertex(std::move(v0Owner)));

    auto v1 = std::make_unique<g2o::VertexSE3>();
    v1->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v1->setId(1);
    ASSERT_TRUE(mOptimizer.addVertex(std::move(v1)));

    auto v2 = std::make_unique<g2o::VertexSE3>();
    v2->setEstimate(
        Eigen::Transform<double, 3, 1>(Eigen::Translation<double, 3>(0, 0, 0)));
    v2->setId(2);
    ASSERT_TRUE(mOptimizer.addVertex(std::move(v2)));

    // Add edges
    auto e1 = std::make_unique<g2o::EdgeSE3>();
    e1->vertices()[0] = mOptimizer.vertex(0);
    e1->vertices()[1] = mOptimizer.vertex(1);
    e1->setMeasurement(
        Eigen::Isometry3d(Eigen::Translation<double, 3>(1, 0, 0)));
    e1->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
    ASSERT_TRUE(mOptimizer.addEdge(std::move(e1)));

    auto e2 = std::make_unique<g2o::EdgeSE3>();
    e2->vertices()[0] = mOptimizer.vertex(1);
    e2->vertices()[1] = mOptimizer.vertex(2);
    e2->setMeasurement(
        Eigen::Isometry3d(Eigen::Translation<double, 3>(0, 1, 0)));
    e2->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
    ASSERT_TRUE(mOptimizer.addEdge(std::move(e2)));

    auto e3 = std::make_unique<g2o::EdgeSE3>();
    e3->vertices()[0] = mOptimizer.vertex(2);
    e3->vertices()[1] = mOptimizer.vertex(0);
    e3->setMeasurement(
        Eigen::Isometry3d(Eigen::Translation<double, 3>(-0.8, -0.7, 0.1)));
    e3->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
    ASSERT_TRUE(mOptimizer.addEdge(std::move(e3)));

    v0->setFixed(true);

    // mOptimizer.setVerbose(true);
    mOptimizer.initializeOptimization();
    mOptimizer.computeInitialGuess();
    mOptimizer.computeActiveErrors();
    int iter = mOptimizer.optimize(10);
    EXPECT_GT(iter, 0);

    mOptimizer.clear();
    EXPECT_TRUE(mOptimizer.vertices().empty());
    EXPECT_TRUE(mOptimizer.edges().empty());
  }
}
