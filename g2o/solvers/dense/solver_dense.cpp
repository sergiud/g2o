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

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l> >

#define ALLOC_DENSE(s, p, l)                                          \
  if (1) {                                                            \
    ngx::Log("# Using DENSE poseDim {} landMarkDim {}", p, l);        \
    DIM_TO_SOLVER(p, l)::LinearSolverType* linearSolver =             \
        new LinearSolverDense<DIM_TO_SOLVER(p, l)::PoseMatrixType>(); \
    s = new DIM_TO_SOLVER(p, l)(linearSolver);                        \
  } else                                                              \
    (void)0

using namespace std;

G2O_START_NAMESPACE

static OptimizationAlgorithm* createSolver(const std::string& fullSolverName) {
  g2o::Solver* s = nullptr;

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

  OptimizationAlgorithm* snl = nullptr;
  if (methodName == "gn") {
    snl = new OptimizationAlgorithmGaussNewton(s);
  } else if (methodName == "lm") {
    snl = new OptimizationAlgorithmLevenberg(s);
  }

  return snl;
}

class DenseSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  DenseSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  virtual OptimizationAlgorithm* construct() {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(dense);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense, new DenseSolverCreator(OptimizationAlgorithmProperty(
                  "gn_dense", "Gauss-Newton: Dense solver (variable blocksize)",
                  "Dense", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense3_2,
    new DenseSolverCreator(OptimizationAlgorithmProperty(
        "gn_dense3_2", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense6_3,
    new DenseSolverCreator(OptimizationAlgorithmProperty(
        "gn_dense6_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_dense7_3,
    new DenseSolverCreator(OptimizationAlgorithmProperty(
        "gn_dense7_3", "Gauss-Newton: Dense solver (fixed blocksize)", "Dense",
        true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense, new DenseSolverCreator(OptimizationAlgorithmProperty(
                  "lm_dense", "Levenberg: Dense solver (variable blocksize)",
                  "Dense", false, -1, -1)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense3_2, new DenseSolverCreator(OptimizationAlgorithmProperty(
                     "lm_dense3_2", "Levenberg: Dense solver (fixed blocksize)",
                     "Dense", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense6_3, new DenseSolverCreator(OptimizationAlgorithmProperty(
                     "lm_dense6_3", "Levenberg: Dense solver (fixed blocksize)",
                     "Dense", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_dense7_3, new DenseSolverCreator(OptimizationAlgorithmProperty(
                     "lm_dense7_3", "Levenberg: Dense solver (fixed blocksize)",
                     "Dense", true, 7, 3)));

G2O_END_NAMESPACE
