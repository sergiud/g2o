// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <memory>
#include <utility>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_PCG(s, p, l)                                              \
  if (1) {                                                              \
    ngx::Log("# Using PCG poseDim {} landMarkDim {}", p, l);            \
    auto linearSolver = std::make_unique<                               \
        LinearSolverPCG<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();        \
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

  if (solverName == "pcg") {
    ALLOC_PCG(s, -1, -1);
  } else if (solverName == "pcg3_2") {
    ALLOC_PCG(s, 3, 2);
  } else if (solverName == "pcg6_3") {
    ALLOC_PCG(s, 6, 3);
  } else if (solverName == "pcg7_3") {
    ALLOC_PCG(s, 7, 3);
  }

  std::unique_ptr<OptimizationAlgorithm> snl;
  if (methodName == "gn") {
    snl = std::make_unique<OptimizationAlgorithmGaussNewton>(std::move(s));
  } else if (methodName == "lm") {
    snl = std::make_unique<OptimizationAlgorithmLevenberg>(std::move(s));
  }

  return snl;
}

class PCGSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  PCGSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(pcg);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                "gn_pcg",
                "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner "
                "(variable blocksize)",
                "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg3_2, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "gn_pcg3_2",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg6_3, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "gn_pcg6_3",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg7_3, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "gn_pcg7_3",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                "lm_pcg",
                "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                "(variable blocksize)",
                "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg3_2, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "lm_pcg3_2",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg6_3, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "lm_pcg6_3",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg7_3, std::make_unique<PCGSolverCreator>(OptimizationAlgorithmProperty(
                   "lm_pcg7_3",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 7, 3)));
G2O_END_NAMESPACE
