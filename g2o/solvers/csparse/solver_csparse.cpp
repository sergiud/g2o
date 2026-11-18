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
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <memory>
#include <utility>

// #define ADD_SCALAR_ORDERING

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

  if (solverName == "var") {
    ALLOC_CSPARSE(s, -1, -1, false);
  } else if (solverName == "fix3_2") {
    ALLOC_CSPARSE(s, 3, 2, true);
  } else if (solverName == "fix6_3") {
    ALLOC_CSPARSE(s, 6, 3, true);
  } else if (solverName == "fix7_3") {
    ALLOC_CSPARSE(s, 7, 3, true);
  }
#ifdef ADD_SCALAR_ORDERING
  else if (solverName == "fix3_2_scalar") {
    ALLOC_CSPARSE(s, 3, 2, false);
  } else if (solverName == "fix6_3_scalar") {
    ALLOC_CSPARSE(s, 6, 3, false);
  } else if (solverName == "fix7_3_scalar") {
    ALLOC_CSPARSE(s, 7, 3, false);
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

class CSparseSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  CSparseSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(csparse);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_var,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_var",
        "Gauss-Newton: Cholesky solver using CSparse (variable blocksize)",
        "CSparse", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3",
        "Gauss-Newton: Cholesky solver using CSparse (fixed blocksize)",
        "CSparse", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_var,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_var",
        "Levenberg: Cholesky solver using CSparse (variable blocksize)",
        "CSparse", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3",
        "Levenberg: Cholesky solver using CSparse (fixed blocksize)", "CSparse",
        true, 7, 3)));

#ifdef ADD_SCALAR_ORDERING
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2_scalar",
        "Gauss-Newton: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3_scalar",
        "Gauss-Newton: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3_scalar",
        "Gauss-Newton: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2_scalar",
        "Levenberg: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3_scalar",
        "Levenberg: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3_scalar,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3_scalar",
        "Levenberg: Cholesky solver using CSparse (fixed "
        "blocksize, scalar ordering)",
        "CSparse", true, 7, 3)));
#endif

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    dl_var,
    std::make_unique<CSparseSolverCreator>(OptimizationAlgorithmProperty(
        "dl_var", "Dogleg: Cholesky solver using CSparse (variable blocksize)",
        "CSparse", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_END_NAMESPACE
