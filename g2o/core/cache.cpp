// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/cache.h>
#include <g2o/core/factory.h>

#include <iostream>

G2O_START_NAMESPACE
using namespace std;

Cache::CacheKey::CacheKey() : _type(), _parameters() {}

Cache::CacheKey::CacheKey(const std::string& type_,
                          const ParameterVector& parameters_)
    : _type(type_), _parameters(parameters_) {}

Cache::Cache(CacheContainer* container_, const ParameterVector& parameters_)
    : _updateNeeded(true), _parameters(parameters_), _container(container_) {}

bool Cache::CacheKey::operator<(const Cache::CacheKey& c) const {
  if (_type < c._type)
    return true;
  else if (c._type < _type)
    return false;
  return std::lexicographical_compare(_parameters.begin(), _parameters.end(),
                                      c._parameters.begin(),
                                      c._parameters.end());
}

OptimizableGraph::Vertex* Cache::vertex() {
  if (container() != nullptr) return container()->vertex();
  return nullptr;
}

OptimizableGraph* Cache::graph() {
  if (container() != nullptr) return container()->graph();
  return nullptr;
}

CacheContainer* Cache::container() { return _container; }

ParameterVector& Cache::parameters() { return _parameters; }

Cache::CacheKey Cache::key() const {
  Factory* factory = Factory::instance();
  return CacheKey(factory->tag(this), _parameters);
};

void Cache::update() {
  if (!_updateNeeded) return;
  for (auto& _parentCache : _parentCaches) {
    _parentCache->update();
  }
  updateImpl();
  _updateNeeded = false;
}

Cache* Cache::installDependency(const std::string& type_,
                                const std::vector<int>& parameterIndices) {
  ParameterVector pv(parameterIndices.size());
  for (size_t i = 0; i < parameterIndices.size(); i++) {
    if (parameterIndices[i] < 0 ||
        parameterIndices[i] >= (int)_parameters.size())
      return nullptr;
    pv[i] = _parameters[parameterIndices[i]];
  }
  CacheKey k(type_, pv);
  if (container() == nullptr) return nullptr;
  Cache* c = container()->findCache(k);
  if (c == nullptr) {
    c = container()->createCache(k);
  }
  if (c != nullptr) _parentCaches.push_back(c);
  return c;
}

bool Cache::resolveDependancies() { return true; }

CacheContainer::CacheContainer(OptimizableGraph::Vertex* vertex_) {
  _vertex = vertex_;
}

Cache* CacheContainer::findCache(const Cache::CacheKey& key) {
  auto it = find(key);
  if (it == end()) return nullptr;
  return it->second;
}

Cache* CacheContainer::createCache(const Cache::CacheKey& key) {
  Factory* f = Factory::instance();
  HyperGraph::HyperGraphElement* e = f->construct(key.type());
  if (e == nullptr) {
    cerr << __PRETTY_FUNCTION__ << endl;
    cerr << "fatal error in creating cache of type " << key.type() << endl;
    return nullptr;
  }
  Cache* c = dynamic_cast<Cache*>(e);
  if (c == nullptr) {
    cerr << __PRETTY_FUNCTION__ << endl;
    cerr << "fatal error in creating cache of type " << key.type() << endl;
    return nullptr;
  }
  c->_container = this;
  c->_parameters = key._parameters;
  if (c->resolveDependancies()) {
    insert(make_pair(key, c));
    c->update();
    return c;
  }
  return nullptr;
}

OptimizableGraph::Vertex* CacheContainer::vertex() { return _vertex; }

OptimizableGraph* CacheContainer::graph() {
  if (_vertex != nullptr) return _vertex->graph();
  return nullptr;
}

void CacheContainer::update() {
  for (auto& it : *this) {
    (it.second)->update();
  }
  _updateNeeded = false;
}

void CacheContainer::setUpdateNeeded(bool needUpdate) {
  _updateNeeded = needUpdate;
  for (auto& it : *this) {
    (it.second)->_updateNeeded = needUpdate;
  }
}

CacheContainer::~CacheContainer() {
  for (auto& it : *this) {
    delete (it.second);
  }
}

G2O_END_NAMESPACE
