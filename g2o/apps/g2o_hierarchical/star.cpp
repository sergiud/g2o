// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "star.h"

#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/stuff/logger.h>

G2O_START_NAMESPACE
using namespace std;

Star::Star(int level, SparseOptimizer* optimizer)
    : _level(level), _optimizer(optimizer) {}

bool Star::labelStarEdges(int iterations, EdgeLabeler* labeler) {
  // mark all vertices in the lowLevelEdges as floating
  bool ok = true;
  std::set<OptimizableGraph::Vertex*> vset;
  for (auto e : _lowLevelEdges) {
    for (auto& i : e->vertices()) {
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(i);
      v->setFixed(false);
      vset.insert(v);
    }
  }
  for (auto v : vset) {
    v->push();
  }

  // fix all vertices in the gauge
  // cerr << "fixing gauge: ";
  for (auto it : _gauge) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
    // cerr << v->id() << " ";
    v->setFixed(true);
  }
  // cerr << endl;
  if (iterations > 0) {
    _optimizer->initializeOptimization(_lowLevelEdges);
    _optimizer->computeInitialGuess();
    int result = _optimizer->optimize(iterations);
    if (result < 1) {
      std::string ids;
      for (auto i : _optimizer->indexMapping()) {
        std::format_to(std::back_inserter(ids), "{} ", i->id());
      }
      std::string gauge;
      for (auto it : _gauge) {
        OptimizableGraph::Vertex* v =
            dynamic_cast<OptimizableGraph::Vertex*>(it);
        std::format_to(std::back_inserter(gauge), "[{} {}] ", v->id(),
                       v->hessianIndex());
      }
      ngx::Log(ngx::Error,
               "!!! optimization failure, vertices num: {} ids: {}, star "
               "size={}, gauge: {}",
               _optimizer->activeVertices().size(), ids, _lowLevelEdges.size(),
               gauge);
      ok = false;
    }
  } else {
    optimizer()->initializeOptimization(_lowLevelEdges);
    // cerr << "guess" << endl;
    // optimizer()->computeInitialGuess();
    // cerr << "solver init" << endl;
    optimizer()->solver()->init();
    // cerr << "structure" << endl;
    OptimizationAlgorithmWithHessian* solverWithHessian =
        dynamic_cast<OptimizationAlgorithmWithHessian*>(optimizer()->solver());
    if (!solverWithHessian->buildLinearStructure())
      ngx::Log(ngx::Error, "failure while building linear structure");
    // cerr << "errors" << endl;
    optimizer()->computeActiveErrors();
    // cerr << "system" << endl;
    solverWithHessian->updateLinearSystem();
  }

  std::set<OptimizableGraph::Edge*> star;
  for (auto _starEdge : _starEdges) {
    star.insert(dynamic_cast<OptimizableGraph::Edge*>(_starEdge));
  }
  if (ok) {
    int result = labeler->labelEdges(star);
    if (result < 0) ok = false;
  }
  // release all vertices in the gauge
  for (auto v : vset) {
    v->pop();
  }
  for (auto it : _gauge) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
    v->setFixed(false);
  }

  return ok;
}

G2O_END_NAMESPACE
