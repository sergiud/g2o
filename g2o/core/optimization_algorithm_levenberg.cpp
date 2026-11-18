// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/timeutil.h>

#include <memory>
#include <ostream>
#include <utility>
using namespace std;

G2O_START_NAMESPACE

OptimizationAlgorithmLevenberg::OptimizationAlgorithmLevenberg(
    std::unique_ptr<Solver> solver)
    : OptimizationAlgorithmWithHessian(std::move(solver)) {
  _currentLambda = -1.;
  _tau = 1e-5;
  _goodStepUpperScale = 2. / 3.;
  _goodStepLowerScale = 1. / 3.;
  _userLambdaInit =
      _properties.makeProperty<Property<double> >("initialLambda", 0.);
  _maxTrialsAfterFailure =
      _properties.makeProperty<Property<int> >("maxTrialsAfterFailure", 10);
  _ni = 2.;
  _levenbergIterations = 0;
}

OptimizationAlgorithmLevenberg::~OptimizationAlgorithmLevenberg() = default;

OptimizationAlgorithm::SolverResult OptimizationAlgorithmLevenberg::solve(
    int iteration, bool online) {
  assert(_optimizer && "_optimizer not set");
  assert(_solver->optimizer() == _optimizer &&
         "underlying linear solver operates on different graph");

  if (iteration == 0 &&
      !online) {  // built up the CCS structure, here due to easy time measure
    bool ok = _solver->buildStructure();
    if (!ok) {
      ngx::Log(ngx::Error, "Failure while building CCS structure");
      return OptimizationAlgorithm::Fail;
    }
  }

  double t = get_monotonic_time();
  _optimizer->computeActiveErrors();
  G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
  if (globalStats != nullptr) {
    globalStats->timeResiduals = get_monotonic_time() - t;
    t = get_monotonic_time();
  }

  double currentChi = _optimizer->activeRobustChi2();

  _solver->buildSystem();
  if (globalStats != nullptr) {
    globalStats->timeQuadraticForm = get_monotonic_time() - t;
  }

  // core part of the Levenbarg algorithm
  if (iteration == 0) {
    _currentLambda = computeLambdaInit();
    _ni = 2;
  }

  double rho = 0;
  int& qmax = _levenbergIterations;
  qmax = 0;
  do {
    _optimizer->push();
    if (globalStats != nullptr) {
      globalStats->levenbergIterations++;
      t = get_monotonic_time();
    }
    // update the diagonal of the system matrix
    _solver->setLambda(_currentLambda, true);
    bool ok2 = _solver->solve();
    if (globalStats != nullptr) {
      globalStats->timeLinearSolution += get_monotonic_time() - t;
      t = get_monotonic_time();
    }
    _optimizer->update(_solver->x());
    if (globalStats != nullptr) {
      globalStats->timeUpdate = get_monotonic_time() - t;
    }

    // restore the diagonal
    _solver->restoreDiagonal();

    _optimizer->computeActiveErrors();
    double tempChi = _optimizer->activeRobustChi2();

    if (!ok2) tempChi = std::numeric_limits<double>::max();

    rho = (currentChi - tempChi);
    double scale = computeScale();
    scale += 1e-3;  // make sure it's non-zero :)
    rho /= scale;

    if (rho > 0 && g2o_isfinite(tempChi)) {  // last step was good
      double alpha = 1. - pow((2 * rho - 1), 3);
      // crop lambda between minimum and maximum factors
      alpha = (std::min)(alpha, _goodStepUpperScale);
      double scaleFactor = (std::max)(_goodStepLowerScale, alpha);
      _currentLambda *= scaleFactor;
      _ni = 2;
      currentChi = tempChi;
      _optimizer->discardTop();
    } else {
      _currentLambda *= _ni;
      _ni *= 2;
      _optimizer->pop();  // restore the last state before trying to optimize
      if (!g2o_isfinite(_currentLambda)) break;
    }
    qmax++;
  } while (rho < 0 && qmax < _maxTrialsAfterFailure->value() &&
           !_optimizer->terminate());

  if (qmax == _maxTrialsAfterFailure->value() || rho == 0 ||
      !g2o_isfinite(_currentLambda))
    return Terminate;
  return OK;
}

double OptimizationAlgorithmLevenberg::computeLambdaInit() const {
  if (_userLambdaInit->value() > 0) return _userLambdaInit->value();
  double maxDiagonal = 0.;
  for (auto v : _optimizer->indexMapping()) {
    assert(v);
    int dim = v->dimension();
    for (int j = 0; j < dim; ++j) {
      maxDiagonal = std::max(fabs(v->hessian(j, j)), maxDiagonal);
    }
  }
  return _tau * maxDiagonal;
}

double OptimizationAlgorithmLevenberg::computeScale() const {
  double scale = 0.;
  for (size_t j = 0; j < _solver->vectorSize(); j++) {
    scale +=
        _solver->x()[j] * (_currentLambda * _solver->x()[j] + _solver->b()[j]);
  }
  return scale;
}

void OptimizationAlgorithmLevenberg::setMaxTrialsAfterFailure(int max_trials) {
  _maxTrialsAfterFailure->setValue(max_trials);
}

void OptimizationAlgorithmLevenberg::setUserLambdaInit(double lambda) {
  _userLambdaInit->setValue(lambda);
}

void OptimizationAlgorithmLevenberg::printVerbose(std::ostream& os) const {
  os << "\t schur= " << _solver->schur()
     << "\t lambda= " << FIXED(_currentLambda)
     << "\t levenbergIter= " << _levenbergIterations;
}

G2O_END_NAMESPACE
