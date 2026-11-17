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
#include <g2o/stuff/macros.h>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l> >

#define ALLOC_CSPARSE(s, p, l, blockorder)                                   \
  if (1) {                                                                   \
    std::cerr << "# Using CSparse poseDim " << p << " landMarkDim " << l     \
              << " blockordering " << blockorder << std::endl;               \
    LinearSolverCSparse<DIM_TO_SOLVER(p, l)::PoseMatrixType>* linearSolver = \
        new LinearSolverCSparse<DIM_TO_SOLVER(p, l)::PoseMatrixType>();      \
    linearSolver->setBlockOrdering(blockorder);                              \
    s = new DIM_TO_SOLVER(p, l)(linearSolver);                               \
  } else                                                                     \
    (void)0

G2O_START_NAMESPACE

/**
 * helper function for allocating
 */
static OptimizationAlgorithm* createSolver(const std::string& fullSolverName) {
  if (fullSolverName != "2dlinear") return nullptr;

  g2o::Solver* s = nullptr;
  ALLOC_CSPARSE(s, 3, 2, true);
  OptimizationAlgorithm* snl = nullptr;
  snl = new SolverSLAM2DLinear(s);

  return snl;
}

class SLAM2DLinearSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  SLAM2DLinearSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  virtual OptimizationAlgorithm* construct() {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(slam2d_linear);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    2dlinear,
    new SLAM2DLinearSolverCreator(OptimizationAlgorithmProperty(
        "2dlinear",
        "Solve Orientation + Gauss-Newton: Works only on 2D pose graphs!!",
        "CSparse", false, 3, 3)));

G2O_END_NAMESPACE
