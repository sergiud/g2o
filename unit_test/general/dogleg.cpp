// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <cmath>
#include <memory>
#include <random>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;

namespace {

using Block = g2o::BlockSolverX;

/**
 * Fills a planar graph of poses that observe landmarks, whose ids follow those
 * of the poses, and returns its block solver.
 */
Block* buildGraph(g2o::SparseOptimizer& graph, bool marginalizeLandmarks) {
  constexpr int Poses = 10;
  constexpr int Landmarks = 20;
  constexpr int FirstLandmarkId = 100;
  constexpr double Spacing = 0.5;
  constexpr double Range = 3.;
  constexpr double Distance = 2.;
  constexpr double Noise = 0.3;
  // A seed whose noise needs several steps to converge.
  constexpr unsigned Seed = 42;

  auto block = std::make_unique<Block>(
      std::make_unique<g2o::LinearSolverEigen<Block::PoseMatrixType>>());
  Block* solver = block.get();
  graph.setAlgorithm(
      std::make_unique<g2o::OptimizationAlgorithmDogleg>(std::move(block)));
  std::mt19937 engine(Seed);
  std::normal_distribution<double> noise(0., Noise);
  for (int i = 0; i < Poses; ++i) {
    auto pose = std::make_unique<g2o::VertexSE2>();
    pose->setId(i);
    pose->setEstimate(i == 0 ? g2o::SE2()
                             : g2o::SE2(i + noise(engine), noise(engine),
                                        noise(engine) / 10.));
    pose->setFixed(i == 0);
    graph.addVertex(std::move(pose));
  }
  for (int j = 0; j < Landmarks; ++j) {
    auto landmark = std::make_unique<g2o::VertexPointXY>();
    landmark->setId(FirstLandmarkId + j);
    landmark->setEstimate(
        Eigen::Vector2d(Spacing * j + noise(engine), Distance + noise(engine)));
    landmark->setMarginalized(marginalizeLandmarks);
    graph.addVertex(std::move(landmark));
  }
  for (int i = 0; i < Poses; ++i) {
    for (int j = 0; j < Landmarks; ++j) {
      if (std::abs(Spacing * j - i) > Range) {
        continue;
      }
      auto edge = std::make_unique<g2o::EdgeSE2PointXY>();
      edge->setVertex(0, graph.vertex(i));
      edge->setVertex(1, graph.vertex(FirstLandmarkId + j));
      edge->setMeasurement(Eigen::Vector2d(Spacing * j - i, Distance));
      edge->setInformation(Eigen::Matrix2d::Identity());
      graph.addEdge(std::move(edge));
    }
  }
  graph.initializeOptimization();
  return solver;
}

//! Optimizes the graph with Dogleg and returns its chi2.
double optimizeWithDogleg(bool marginalizeLandmarks) {
  constexpr int Iterations = 8;
  g2o::SparseOptimizer graph;
  buildGraph(graph, marginalizeLandmarks);
  graph.optimize(Iterations);
  return graph.activeChi2();
}

//! Product of the Hessian of the graph with a fixed vector.
Eigen::VectorXd hessianProduct(bool marginalizeLandmarks) {
  g2o::SparseOptimizer graph;
  Block* solver = buildGraph(graph, marginalizeLandmarks);
  graph.computeActiveErrors();
  EXPECT_TRUE(graph.solver()->init());
  EXPECT_TRUE(solver->buildStructure());
  EXPECT_TRUE(solver->buildSystem());
  const Eigen::VectorXd src =
      Eigen::VectorXd::LinSpaced(solver->vectorSize(), -1., 1.);
  Eigen::VectorXd dest = Eigen::VectorXd::Zero(solver->vectorSize());
  solver->multiplyHessian(dest.data(), src.data());
  return dest;
}

}  // namespace

/**
 * Marginalizing the landmarks changes how the system is solved, not its
 * Hessian, so Dogleg takes the same steps and reaches the same optimum.
 */
TEST(Dogleg, MarginalizedLandmarksReachTheSameOptimum) {
  constexpr double Tolerance = 1e-9;

  EXPECT_THAT(optimizeWithDogleg(/*marginalizeLandmarks=*/true),
              DoubleNear(optimizeWithDogleg(/*marginalizeLandmarks=*/false),
                         Tolerance));
}

/**
 * The Schur complement splits the Hessian into blocks, whose product with a
 * vector equals that of the Hessian in one block.
 */
TEST(Dogleg, MultipliesTheCompleteHessian) {
  constexpr double Tolerance = 1e-12;

  EXPECT_THAT(
      hessianProduct(/*marginalizeLandmarks=*/true),
      ::testing::Pointwise(DoubleNear(Tolerance),
                           hessianProduct(/*marginalizeLandmarks=*/false)));
}
