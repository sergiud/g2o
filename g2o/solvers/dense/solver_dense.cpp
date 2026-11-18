// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W.
// Burgard SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/logger.h>

#include <memory>
#include <utility>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_DENSE(s, p, l)                                            \
  if (1) {                                                              \
    ngx::Log("# Using DENSE poseDim {} landMarkDim {}", p, l);          \
    auto linearSolver = std::make_unique<                               \
        LinearSolverDense<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();      \
    s = std::make_unique<DIM_TO_SOLVER(p, l)>(std::move(linearSolver)); \
  } else                                                                \
    (void)0

using namespace std;

G2O_START_NAMESPACE

static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  std::unique_ptr<BlockSolverBase> s;

  string methodName = fullSolverName.substr(0, 2);
  string solverName = fullSolverName.substr(3);

  if (solverName == "dense") {
    ALLOC_DENSE(s, -1, -1);
  } else if (solverName == "dense3_2") {
    ALLOC_DENSE(s, 3, 2);
  } else if (solverName == "dense6_3") {
    ALLOC_DENSE(s, 6, 3);
  } else if (solverName == "dense7_3") {
    ALLOC_DENSE(s, 7, 3);
  }

  std::unique_ptr<OptimizationAlgorithm> snl;
  if (methodName == "gn") {
    snl = std::make_unique<OptimizationAlgorithmGaussNewton>(std::move(s));
  } else if (methodName == "lm") {
    snl = std::make_unique<OptimizationAlgorithmLevenberg>(std::move(s));
  }

  return snl;
}

class DenseSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  DenseSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(dense);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense", "Gauss-Newton: Dense solver (variable blocksize)", "Dense",
        false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense3_2,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense3_2", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense6_3,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense6_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense7_3,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_dense7_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense", "Levenberg: Dense solver (variable blocksize)", "Dense",
        false, -1, -1)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense3_2,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense3_2", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense6_3,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense6_3", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense7_3,
    std::make_unique<DenseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_dense7_3", "Levenberg: Dense solver (fixed blocksize)", "Dense",
        true, 7, 3)));

G2O_END_NAMESPACE
