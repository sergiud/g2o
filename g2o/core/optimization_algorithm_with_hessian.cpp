// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/batch_stats.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>

#include <iostream>
#include <memory>
#include <utility>

using namespace std;

G2O_START_NAMESPACE

OptimizationAlgorithmWithHessian::OptimizationAlgorithmWithHessian(
    std::unique_ptr<Solver> solver)
    : OptimizationAlgorithm(), _solver(std::move(solver)) {
  _writeDebug = _properties.makeProperty<Property<bool> >("writeDebug", true);
}

OptimizationAlgorithmWithHessian::~OptimizationAlgorithmWithHessian() = default;

bool OptimizationAlgorithmWithHessian::init(bool online) {
  assert(_optimizer && "_optimizer not set");
  assert(_solver && "Solver not set");
  _solver->setWriteDebug(_writeDebug->value());
  bool useSchur = false;
  for (auto v : _optimizer->activeVertices()) {
    if (v->marginalized()) {
      useSchur = true;
      break;
    }
  }
  if (useSchur) {
    if (_solver->supportsSchur()) _solver->setSchur(true);
  } else {
    if (_solver->supportsSchur()) _solver->setSchur(false);
  }

  bool initState = _solver->init(_optimizer, online);
  return initState;
}

bool OptimizationAlgorithmWithHessian::computeMarginals(
    SparseBlockMatrix<MatrixXD>& spinv,
    const std::vector<std::pair<int, int> >& blockIndices) {
  return _solver != nullptr ? _solver->computeMarginals(spinv, blockIndices)
                            : false;
}

bool OptimizationAlgorithmWithHessian::buildLinearStructure() {
  return _solver != nullptr ? _solver->buildStructure() : false;
}

void OptimizationAlgorithmWithHessian::updateLinearSystem() {
  if (_solver != nullptr) _solver->buildSystem();
}

bool OptimizationAlgorithmWithHessian::updateStructure(
    const std::vector<HyperGraph::Vertex*>& vset,
    const HyperGraph::EdgeSet& edges) {
  return _solver != nullptr ? _solver->updateStructure(vset, edges) : false;
}

void OptimizationAlgorithmWithHessian::setWriteDebug(bool writeDebug) {
  _writeDebug->setValue(writeDebug);
}

G2O_END_NAMESPACE
