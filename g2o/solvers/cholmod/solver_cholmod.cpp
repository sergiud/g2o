// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <Eigen/Core>
#include <memory>
#include <utility>

// #define ADD_SCALAR_ORDERING

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_CHOLMOD(s, p, l, blockorder)                                    \
  if (1) {                                                                    \
    ngx::Log("# Using CHOLMOD poseDim {} landMarkDim {} blockordering {}", p, \
             l, blockorder);                                                  \
    auto linearSolver = std::make_unique<                                     \
        LinearSolverCholmod<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();          \
    linearSolver->setBlockOrdering(blockorder);                               \
    s = std::make_unique<DIM_TO_SOLVER(p, l)>(std::move(linearSolver));       \
  } else                                                                      \
    (void)0

using namespace std;

G2O_START_NAMESPACE

CholmodExt::CholmodExt() : cholmod_sparse_struct() {
  nzmax = 0;
  nrow = 0;
  ncol = 0;
  p = nullptr;
  i = nullptr;
  nz = nullptr;
  x = nullptr;
  z = nullptr;
  stype = 1;  // upper triangular block only
  itype = CHOLMOD_INT;
  xtype = CHOLMOD_REAL;
  dtype = CHOLMOD_DOUBLE;
  sorted = 1;
  packed = 1;
  columnsAllocated = 0;
}

void CholmodExt::allocateColumns(std::size_t columns) {
  _columnPointers = std::make_unique<int[]>(columns + 1);
  p = _columnPointers.get();
}

void CholmodExt::allocateEntries(std::size_t entries) {
  _rowIndices = std::make_unique<int[]>(entries);
  _values = std::make_unique<double[]>(entries);
  i = _rowIndices.get();
  x = _values.get();
}

static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  std::unique_ptr<BlockSolverBase> s;

  string methodName = fullSolverName.substr(0, 2);
  string solverName = fullSolverName.substr(3);

  if (solverName == "var_cholmod") {
    ALLOC_CHOLMOD(s, -1, -1, false);
  } else if (solverName == "fix3_2_cholmod") {
    ALLOC_CHOLMOD(s, 3, 2, true);
  } else if (solverName == "fix6_3_cholmod") {
    ALLOC_CHOLMOD(s, 6, 3, true);
  } else if (solverName == "fix7_3_cholmod") {
    ALLOC_CHOLMOD(s, 7, 3, true);
  }
#ifdef ADD_SCALAR_ORDERING
  else if (solverName == "fix3_2_cholmod_scalar") {
    ALLOC_CHOLMOD(s, 3, 2, false);
  } else if (solverName == "fix6_3_cholmod_scalar") {
    ALLOC_CHOLMOD(s, 6, 3, false);
  } else if (solverName == "fix7_3_cholmod_scalar") {
    ALLOC_CHOLMOD(s, 7, 3, false);
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

class CholmodSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  CholmodSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(cholmod);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_var_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_var_cholmod",
        "Gauss-Newton: Cholesky solver using CHOLMOD (variable blocksize)",
        "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2_cholmod",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)",
        "CHOLMOD", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3_cholmod",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)",
        "CHOLMOD", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3_cholmod",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize)",
        "CHOLMOD", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_var_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_var_cholmod",
        "Levenberg: Cholesky solver using CHOLMOD (variable blocksize)",
        "CHOLMOD", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2_cholmod",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD",
        true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3_cholmod",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD",
        true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3_cholmod",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize)", "CHOLMOD",
        true, 7, 3)));

#ifdef ADD_SCALAR_ORDERING
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix3_2_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix3_2_cholmod_scalar",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix6_3_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix6_3_cholmod_scalar",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_fix7_3_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "gn_fix7_3_cholmod_scalar",
        "Gauss-Newton: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix3_2_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix3_2_cholmod_scalar",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix6_3_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix6_3_cholmod_scalar",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_fix7_3_cholmod_scalar,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "lm_fix7_3_cholmod_scalar",
        "Levenberg: Cholesky solver using CHOLMOD (fixed blocksize, scalar "
        "ordering)",
        "CHOLMOD", true, 7, 3)));
#endif

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    dl_var_cholmod,
    std::make_unique<CholmodSolverCreator>(OptimizationAlgorithmProperty(
        "dl_var_cholmod",
        "Dogleg: Cholesky solver using CHOLMOD (variable blocksize)", "CHOLMOD",
        false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_END_NAMESPACE
