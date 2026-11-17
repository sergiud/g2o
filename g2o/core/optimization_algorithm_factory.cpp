// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/optimization_algorithm_factory.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <typeinfo>

using namespace std;

G2O_START_NAMESPACE

AbstractOptimizationAlgorithmCreator::AbstractOptimizationAlgorithmCreator(
    const OptimizationAlgorithmProperty& p)
    : _property(p) {}

OptimizationAlgorithmFactory* OptimizationAlgorithmFactory::instance() {
  static OptimizationAlgorithmFactory factoryInstance;
  return &factoryInstance;
}

void OptimizationAlgorithmFactory::registerSolver(
    AbstractOptimizationAlgorithmCreator* c) {
  const string& name = c->property().name;
  auto foundIt = findSolver(name);
  if (foundIt != _creator.end()) {
    _creator.erase(foundIt);
    cerr << "SOLVER FACTORY WARNING: Overwriting Solver creator " << name
         << endl;
    assert(0);
  }
  _creator.emplace_back(c);
}

void OptimizationAlgorithmFactory::unregisterSolver(
    AbstractOptimizationAlgorithmCreator* c) {
  const string& name = c->property().name;
  auto foundIt = findSolver(name);
  if (foundIt != _creator.end()) {
    _creator.erase(foundIt);
  }
}

OptimizationAlgorithm* OptimizationAlgorithmFactory::construct(
    const std::string& name,
    OptimizationAlgorithmProperty& solverProperty) const {
  auto foundIt = findSolver(name);
  if (foundIt != _creator.end()) {
    solverProperty = (*foundIt)->property();
    return (*foundIt)->construct();
  }
  cerr << "SOLVER FACTORY WARNING: Unable to create solver " << name << endl;
  return nullptr;
}

void OptimizationAlgorithmFactory::destroy() {}

void OptimizationAlgorithmFactory::listSolvers(std::ostream& os) const {
  size_t solverNameColumnLength = 0;
  for (const auto& it : _creator)
    solverNameColumnLength =
        std::max(solverNameColumnLength, it->property().name.size());
  solverNameColumnLength += 4;

  for (const auto& it : _creator) {
    const OptimizationAlgorithmProperty& sp = it->property();
    os << sp.name;
    for (size_t i = sp.name.size(); i < solverNameColumnLength; ++i) os << ' ';
    os << sp.desc << endl;
  }
}

OptimizationAlgorithmFactory::CreatorList::const_iterator
OptimizationAlgorithmFactory::findSolver(const std::string& name) const {
  for (auto it = _creator.begin(); it != _creator.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (sp.name == name) return it;
  }
  return _creator.end();
}

OptimizationAlgorithmFactory::CreatorList::iterator
OptimizationAlgorithmFactory::findSolver(const std::string& name) {
  for (auto it = _creator.begin(); it != _creator.end(); ++it) {
    const OptimizationAlgorithmProperty& sp = (*it)->property();
    if (sp.name == name) return it;
  }
  return _creator.end();
}

G2O_END_NAMESPACE
