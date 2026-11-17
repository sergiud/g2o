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
#include <g2o/stuff/macros.h>

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l> >

#define ALLOC_PCG(s, p, l)                                           \
  if (1) {                                                           \
    std::cerr << "# Using PCG poseDim " << p << " landMarkDim " << l \
              << std::endl;                                          \
    DIM_TO_SOLVER(p, l)::LinearSolverType* linearSolver =            \
        new LinearSolverPCG<DIM_TO_SOLVER(p, l)::PoseMatrixType>();  \
    s = new DIM_TO_SOLVER(p, l)(linearSolver);                       \
  } else                                                             \
    (void)0

using namespace std;

G2O_START_NAMESPACE

static OptimizationAlgorithm* createSolver(const std::string& fullSolverName) {
  g2o::Solver* s = nullptr;

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

  OptimizationAlgorithm* snl = nullptr;
  if (methodName == "gn") {
    snl = new OptimizationAlgorithmGaussNewton(s);
  } else if (methodName == "lm") {
    snl = new OptimizationAlgorithmLevenberg(s);
  }

  return snl;
}

class PCGSolverCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  PCGSolverCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  virtual OptimizationAlgorithm* construct() {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(pcg);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg, new PCGSolverCreator(OptimizationAlgorithmProperty(
                "gn_pcg",
                "Gauss-Newton: PCG solver using block-Jacobi pre-conditioner "
                "(variable blocksize)",
                "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg3_2, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "gn_pcg3_2",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg6_3, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "gn_pcg6_3",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    gn_pcg7_3, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "gn_pcg7_3",
                   "Gauss-Newton: PCG solver using block-Jacobi "
                   "pre-conditioner (fixed blocksize)",
                   "PCG", true, 7, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg, new PCGSolverCreator(OptimizationAlgorithmProperty(
                "lm_pcg",
                "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                "(variable blocksize)",
                "PCG", false, Eigen::Dynamic, Eigen::Dynamic)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg3_2, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "lm_pcg3_2",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg6_3, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "lm_pcg6_3",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 6, 3)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    lm_pcg7_3, new PCGSolverCreator(OptimizationAlgorithmProperty(
                   "lm_pcg7_3",
                   "Levenberg: PCG solver using block-Jacobi pre-conditioner "
                   "(fixed blocksize)",
                   "PCG", true, 7, 3)));
G2O_END_NAMESPACE
