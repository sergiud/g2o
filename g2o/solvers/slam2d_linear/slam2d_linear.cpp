// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_property.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/slam2d_linear/solver_slam2d_linear.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <memory>
#include <utility>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_CSPARSE(s, p, l, blockorder)                                    \
  if (1) {                                                                    \
    ngx::Log("# Using CSparse poseDim {} landMarkDim {} blockordering {}", p, \
             l, blockorder);                                                  \
    auto linearSolver = std::make_unique<                                     \
        LinearSolverCSparse<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();          \
    linearSolver->setBlockOrdering(blockorder);                               \
    s = std::make_unique<DIM_TO_SOLVER(p, l)>(std::move(linearSolver));       \
  } else                                                                      \
    (void)0

G2O_START_NAMESPACE

/**
 * helper function for allocating
 */
static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  if (fullSolverName != "2dlinear") return nullptr;

  std::unique_ptr<BlockSolverBase> s;
  ALLOC_CSPARSE(s, 3, 2, true);
  return std::make_unique<SolverSLAM2DLinear>(std::move(s));
}

class SLAM2DLinearSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  SLAM2DLinearSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(slam2d_linear);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    2dlinear,
    std::make_unique<SLAM2DLinearSolverCreator>(OptimizationAlgorithmProperty(
        "2dlinear",
        "Solve Orientation + Gauss-Newton: Works only on 2D pose graphs!!",
        "CSparse", false, 3, 3)));

G2O_END_NAMESPACE
