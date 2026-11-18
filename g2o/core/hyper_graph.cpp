// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2022 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/hyper_graph.h>

#include <cassert>
#include <queue>

G2O_START_NAMESPACE

HyperGraph::Data::Data() {
  _next = nullptr;
  _dataContainer = nullptr;
}

HyperGraph::Data::~Data() { delete _next; }

HyperGraph::Vertex::Vertex(int id) : _id(id) {}

HyperGraph::Vertex::~Vertex() = default;

HyperGraph::Edge::Edge(int id) : _id(id) {}

HyperGraph::HyperGraphElement::~HyperGraphElement() = default;

int HyperGraph::Edge::numUndefinedVertices() const {
  int undefined = 0;
  for (auto _vertice : _vertices) {
    if (_vertice == nullptr) undefined++;
  }
  return undefined;
}

void HyperGraph::Edge::resize(size_t size) { _vertices.resize(size, nullptr); }

void HyperGraph::Edge::setId(int id) { _id = id; }

HyperGraph::Vertex* HyperGraph::vertex(int id) {
  auto it = _vertices.find(id);
  if (it == _vertices.end()) return nullptr;
  return it->second.get();
}

const HyperGraph::Vertex* HyperGraph::vertex(int id) const {
  auto it = _vertices.find(id);
  if (it == _vertices.end()) return nullptr;
  return it->second.get();
}

bool HyperGraph::addVertex(Vertex* v) {
  Vertex* vn = vertex(v->id());
  if (vn != nullptr) return false;
  _vertices.emplace(v->id(), std::unique_ptr<Vertex>(v));
  return true;
}

/**
 * changes the id of a vertex already in the graph, and updates the bookkeeping
 @ returns false if the vertex is not in the graph;
*/
bool HyperGraph::changeId(Vertex* v, int newId) {
  Vertex* v2 = vertex(v->id());
  if (v != v2) return false;
  if (newId != v->id() && vertex(newId) != nullptr) return false;
  auto it = _vertices.find(v->id());
  std::unique_ptr<Vertex> owner = std::move(it->second);
  _vertices.erase(it);
  v->setId(newId);
  _vertices.emplace(newId, std::move(owner));
  return true;
}

bool HyperGraph::addEdge(Edge* e) {
  for (auto& it : e->vertices()) {
    if (it == nullptr) return false;
  }
  // An edge the graph already owns must not get a second owner.
  if (_edges.find(e) != _edges.end()) return false;
  _edges.emplace(e);
  for (auto it = e->vertices().begin(); it != e->vertices().end(); ++it) {
    Vertex* v = *it;
    v->edges().insert(e);
  }
  return true;
}

bool HyperGraph::setEdgeVertex(HyperGraph::Edge* e, int pos,
                               HyperGraph::Vertex* v) {
  Vertex* vOld = e->vertex(pos);
  if (vOld != nullptr) vOld->edges().erase(e);
  e->setVertex(pos, v);
  if (v != nullptr) v->edges().insert(e);
  return true;
}

bool HyperGraph::mergeVertices(Vertex* vBig, Vertex* vSmall, bool erase) {
  auto it = _vertices.find(vBig->id());
  if (it == _vertices.end()) return false;

  it = _vertices.find(vSmall->id());
  if (it == _vertices.end()) return false;

  EdgeSet tmp(vSmall->edges());
  bool ok = true;
  for (auto e : tmp) {
    for (size_t i = 0; i < e->vertices().size(); i++) {
      Vertex* v = e->vertex(i);
      if (v == vSmall) ok &= static_cast<int>(setEdgeVertex(e, i, vBig));
    }
  }
  if (erase) removeVertex(vSmall);
  return ok;
}

bool HyperGraph::detachVertex(Vertex* v) {
  auto it = _vertices.find(v->id());
  if (it == _vertices.end()) return false;
  assert(it->second.get() == v);
  EdgeSet tmp(v->edges());
  for (auto e : tmp) {
    for (size_t i = 0; i < e->vertices().size(); i++) {
      if (v == e->vertex(i)) setEdgeVertex(e, i, nullptr);
    }
  }
  return true;
}

bool HyperGraph::removeVertex(Vertex* v, bool detach) {
  if (detach) {
    bool result = detachVertex(v);
    if (!result) {
      assert(0 && "inconsistency in detaching vertex, ");
    }
  }
  auto it = _vertices.find(v->id());
  if (it == _vertices.end()) return false;
  assert(it->second.get() == v);
  // remove all edges which are entering or leaving v;
  EdgeSet tmp(v->edges());
  for (auto e : tmp) {
    if (!removeEdge(e)) {
      assert(0 && "error in erasing vertex");
    }
  }
  _vertices.erase(it);
  return true;
}

bool HyperGraph::removeEdge(Edge* e) { return releaseEdge(e) != nullptr; }

std::unique_ptr<HyperGraph::Edge> HyperGraph::releaseEdge(Edge* e) {
  auto owner = _edges.find(e);
  if (owner == _edges.end()) return nullptr;
  for (auto vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
    Vertex* v = *vit;
    if (v == nullptr) continue;
    auto it = v->edges().find(e);
    assert(it != v->edges().end());
    v->edges().erase(it);
  }
  return std::move(_edges.extract(owner).value());
}

HyperGraph::HyperGraph() {}

void HyperGraph::clear() {
  _edges.clear();
  _vertices.clear();
}

HyperGraph::EdgeSet HyperGraph::edgePointers() const {
  EdgeSet edges;
  for (const auto& edge : _edges) {
    edges.insert(edge.get());
  }
  return edges;
}

HyperGraph::~HyperGraph() { clear(); }

G2O_END_NAMESPACE
