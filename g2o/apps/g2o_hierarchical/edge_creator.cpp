// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "edge_creator.h"

#include <g2o/core/factory.h>
#include <g2o/stuff/logger.h>

G2O_START_NAMESPACE

using namespace std;

bool EdgeCreator::addAssociation(const std::string& vertexTypes,
                                 const std::string& edgeType,
                                 const std::vector<int>& parameterIds) {
  auto it = _vertexToEdgeMap.find(vertexTypes);
  if (it != _vertexToEdgeMap.end())
    it->second = edgeType;
  else
    _vertexToEdgeMap.insert(
        make_pair(vertexTypes, EdgeCreatorEntry(edgeType, parameterIds)));
  return true;
}

bool EdgeCreator::addAssociation(const std::string& vertexTypes,
                                 const std::string& edgeType) {
  return addAssociation(vertexTypes, edgeType, std::vector<int>());
}

bool EdgeCreator::removeAssociation(std::string vertexTypes) {
  auto it = _vertexToEdgeMap.find(vertexTypes);
  if (it == _vertexToEdgeMap.end()) return false;
  _vertexToEdgeMap.erase(it);
  return true;
}

OptimizableGraph::Edge* EdgeCreator::createEdge(
    std::vector<OptimizableGraph::Vertex*>& vertices) {
  std::stringstream key;
  Factory* factory = Factory::instance();
  for (auto& vertice : vertices) {
    key << factory->tag(vertice) << ";";
  }
  auto it = _vertexToEdgeMap.find(key.str());
  if (it == _vertexToEdgeMap.end()) {
    ngx::Log(ngx::Error, "no thing in factory: {}", key.str());
    return nullptr;
  }
  HyperGraph::HyperGraphElement* element =
      factory->construct(it->second._edgeTypeName);
  if (element == nullptr) {
    ngx::Log(ngx::Error, "no thing can be created");
    return nullptr;
  }
  OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(element);
  assert(it->second._parameterIds.size() == e->numParameters());
  for (size_t i = 0; i < it->second._parameterIds.size(); i++) {
    if (!e->setParameterId(i, it->second._parameterIds[i])) {
      ngx::Log(ngx::Error, "no thing in good for setting params");
      return nullptr;
    }
  }
  assert(e);
  for (size_t i = 0; i < vertices.size(); i++) e->vertices()[i] = vertices[i];
  return e;
}

G2O_END_NAMESPACE
