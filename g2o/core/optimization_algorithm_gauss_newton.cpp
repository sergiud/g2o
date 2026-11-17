// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/timeutil.h>

#include <ostream>

using namespace std;

G2O_START_NAMESPACE

OptimizationAlgorithmGaussNewton::OptimizationAlgorithmGaussNewton(
    Solver* solver)
    : OptimizationAlgorithmWithHessian(solver) {}

OptimizationAlgorithmGaussNewton::~OptimizationAlgorithmGaussNewton() = default;

OptimizationAlgorithm::SolverResult OptimizationAlgorithmGaussNewton::solve(
    int iteration, bool online) {
  assert(_optimizer && "_optimizer not set");
  assert(_solver->optimizer() == _optimizer &&
         "underlying linear solver operates on different graph");
  bool ok = true;

  // here so that correct component for max-mixtures can be computed before the
  // build structure
  double t = get_monotonic_time();
  _optimizer->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats != nullptr) {
    globalStats->timeResiduals = get_monotonic_time() - t;
  }

  if (iteration == 0 &&
      !online) {  // built up the CCS structure, here due to easy time measure
    ok = _solver->buildStructure();
    if (!ok) {
      ngx::Log(ngx::Error, "Failure while building CCS structure");
      return OptimizationAlgorithm::Fail;
    }
  }

  t = get_monotonic_time();
  _solver->buildSystem();
  if (globalStats != nullptr) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  ok = _solver->solve();
  if (globalStats != nullptr) {
    globalStats->timeLinearSolution = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  _optimizer->update(_solver->x());
  if (globalStats != nullptr) {
    globalStats->timeUpdate = get_monotonic_time() - t;
  }
  if (ok)
    return OK;
  else
    return Fail;
}

void OptimizationAlgorithmGaussNewton::printVerbose(std::ostream& os) const {
  os << "\t schur= " << _solver->schur();
}

G2O_END_NAMESPACE
