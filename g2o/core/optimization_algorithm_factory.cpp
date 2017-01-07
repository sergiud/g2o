// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <g2o/core/optimization_algorithm_factory.h>

#include <algorithm>
#include <cassert>
#include <iostream>
#include <typeinfo>

using namespace std;

namespace g2o {

  AbstractOptimizationAlgorithmCreator::AbstractOptimizationAlgorithmCreator(const OptimizationAlgorithmProperty& p) :
    _property(p)
  {
  }

  OptimizationAlgorithmFactory* OptimizationAlgorithmFactory::instance()
  {
    static OptimizationAlgorithmFactory factoryInstance;
    return &factoryInstance;
  }

  void OptimizationAlgorithmFactory::registerSolver(AbstractOptimizationAlgorithmCreator* c)
  {
    const string& name = c->property().name;
    auto foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      _creator.erase(foundIt);
      cerr << "SOLVER FACTORY WARNING: Overwriting Solver creator " << name << endl;
      assert(0);
    }
    _creator.push_back(std::unique_ptr<AbstractOptimizationAlgorithmCreator>(c));
  }

  void OptimizationAlgorithmFactory::unregisterSolver(AbstractOptimizationAlgorithmCreator* c)
  {
    const string& name = c->property().name;
    auto foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      _creator.erase(foundIt);
    }
  }

  OptimizationAlgorithm* OptimizationAlgorithmFactory::construct(const std::string& name, OptimizationAlgorithmProperty& solverProperty) const
  {
    auto foundIt = findSolver(name);
    if (foundIt != _creator.end()) {
      solverProperty = (*foundIt)->property();
      return (*foundIt)->construct();
    }
    cerr << "SOLVER FACTORY WARNING: Unable to create solver " << name << endl;
    return nullptr;
  }

  void OptimizationAlgorithmFactory::destroy()
  {
  }

  void OptimizationAlgorithmFactory::listSolvers(std::ostream& os) const
  {
    size_t solverNameColumnLength = 0;
    for (const auto & it : _creator)
      solverNameColumnLength = std::max(solverNameColumnLength, it->property().name.size());
    solverNameColumnLength += 4;

    for (const auto & it : _creator) {
      const OptimizationAlgorithmProperty& sp = it->property();
      os << sp.name;
      for (size_t i = sp.name.size(); i < solverNameColumnLength; ++i)
        os << ' ';
      os << sp.desc << endl;
    }
  }

  OptimizationAlgorithmFactory::CreatorList::const_iterator OptimizationAlgorithmFactory::findSolver(const std::string& name) const
  {
    for (auto it = _creator.begin(); it != _creator.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

  OptimizationAlgorithmFactory::CreatorList::iterator OptimizationAlgorithmFactory::findSolver(const std::string& name)
  {
    for (auto it = _creator.begin(); it != _creator.end(); ++it) {
      const OptimizationAlgorithmProperty& sp = (*it)->property();
      if (sp.name == name)
        return it;
    }
    return _creator.end();
  }

} // namespace g2o
