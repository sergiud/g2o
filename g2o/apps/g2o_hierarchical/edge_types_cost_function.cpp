// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "edge_types_cost_function.h"

#include <g2o/core/factory.h>

#include <limits>

G2O_START_NAMESPACE

EdgeTypesCostFunction::EdgeTypesCostFunction(std::string edgeTag,
                                             std::string vertexTag, int level)
    : _edgeTag(edgeTag),
      _vertexTag(vertexTag),
      _factory(Factory::instance()),
      _level(level) {}

double EdgeTypesCostFunction::operator()(HyperGraph::Edge* e_,
                                         HyperGraph::Vertex* from,
                                         HyperGraph::Vertex* to) {
  OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(e_);
  if (e->level() == _level && _factory->tag(e) == _edgeTag &&
      _factory->tag(from) == _vertexTag && _factory->tag(to) == _vertexTag) {
    return 1.;
  }
  return std::numeric_limits<double>::max();
}

G2O_END_NAMESPACE
