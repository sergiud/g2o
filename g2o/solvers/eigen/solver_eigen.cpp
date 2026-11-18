// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <memory>
#include <utility>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_EIGEN_SPARSE_CHOLESKY(s, p, l, blockorder)                       \
  if (1) {                                                                     \
    ngx::Log(                                                                  \
        "# Using EigenSparseCholesky poseDim {} landMarkDim {} blockordering " \
        "{}",                                                                  \
        p, l, blockorder);                                                     \
    auto linearSolver = std::make_unique<                                      \
        LinearSolverEigen<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();             \
    linearSolver->setBlockOrdering(blockorder);                                \
    s = std::make_unique<DIM_TO_SOLVER(p, l)>(std::move(linearSolver));        \
  } else                                                                       \
    (void)0

using namespace std;

G2O_START_NAMESPACE

/**
 * helper function for allocating
 */
static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  std::unique_ptr<BlockSolverBase> s;

  string methodName = fullSolverName.substr(0, 2);
  string solverName = fullSolverName.substr(3);

  if (solverName == "var_eigen") {
    ALLOC_EIGEN_SPARSE_CHOLESKY(s, -1, -1, true);
  }
#if 0
    else if (solverName == "fix3_2_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 3, 2, true);
    }
    else if (solverName == "fix6_3_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 6, 3, true);
    }
    else if (solverName == "fix7_3_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 7, 3, true);
    }
    else if (solverName == "fix3_2_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 3, 2, false);
    }
    else if (solverName == "fix6_3_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 6, 3, false);
    }
    else if (solverName == "fix7_3_scalar_eigen") {
      ALLOC_EIGEN_SPARSE_CHOLESKY(s, 7, 3, false);
    }
#endif

  std::unique_ptr<OptimizationAlgorithm> snl;
  if (methodName == "gn") {
    snl = std::make_unique<OptimizationAlgorithmGaussNewton>(std::move(s));
  } else if (methodName == "lm") {
    snl = std::make_unique<OptimizationAlgorithmLevenberg>(std::move(s));
  } else if (methodName == "dl") {
    snl = std::make_unique<OptimizationAlgorithmDogleg>(std::move(s));
  }

  return snl;
}

class EigenSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  EigenSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(eigen);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_var_eigen,
    std::make_unique<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "gn_var_eigen",
        "Gauss-Newton: Cholesky solver using Eigen's Sparse "
        "Cholesky methods (variable blocksize)",
        "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_var_eigen,
    std::make_unique<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "lm_var_eigen",
        "Levenberg: Cholesky solver using Eigen's Sparse "
        "Cholesky methods (variable blocksize)",
        "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    dl_var_eigen,
    std::make_unique<EigenSolverCreator>(OptimizationAlgorithmProperty(
        "dl_var_eigen",
        "Dogleg: Cholesky solver using Eigen's Sparse Cholesky "
        "methods (variable blocksize)",
        "Eigen", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_END_NAMESPACE
