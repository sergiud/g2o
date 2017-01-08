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

#include <g2o/core/hyper_graph.h>

#include <cassert>
#include <queue>

namespace g2o {

  HyperGraph::Data::Data() {
    _next = nullptr;
    _dataContainer = nullptr;
  }

  HyperGraph::Data::~Data() {
    delete _next;
  }

  HyperGraph::Vertex::Vertex(int id) : _id(id)
  {
  }

  HyperGraph::Vertex::~Vertex() = default;

  HyperGraph::Edge::Edge(int id) : _id(id)
  {
  }

  HyperGraph::HyperGraphElement::~HyperGraphElement() = default;

  int HyperGraph::Edge::numUndefinedVertices() const{
    int undefined=0;
    for (auto _vertice : _vertices){
      if (_vertice == nullptr)
	undefined++;
    }
    return undefined;
  }

  void HyperGraph::Edge::resize(size_t size)
  {
    _vertices.resize(size, nullptr);
  }

  void HyperGraph::Edge::setId(int id)
  {
    _id = id;
  }

  HyperGraph::Vertex* HyperGraph::vertex(int id)
  {
    auto it=_vertices.find(id);
    if (it==_vertices.end())
      return nullptr;
    return it->second;
  }

  const HyperGraph::Vertex* HyperGraph::vertex(int id) const
  {
    auto it=_vertices.find(id);
    if (it==_vertices.end())
      return nullptr;
    return it->second;
  }

  bool HyperGraph::addVertex(Vertex* v)
  {
    Vertex* vn=vertex(v->id());
    if (vn != nullptr)
      return false;
    _vertices.insert( std::make_pair(v->id(),v) );
    return true;
  }

  /**
   * changes the id of a vertex already in the graph, and updates the bookkeeping
   @ returns false if the vertex is not in the graph;
  */
  bool HyperGraph::changeId(Vertex* v, int newId){
    Vertex* v2 = vertex(v->id());
    if (v != v2)
      return false;
    _vertices.erase(v->id());
    v->setId(newId);
    _vertices.insert(std::make_pair(v->id(), v));
    return true;
  }

  bool HyperGraph::addEdge(Edge* e)
  {
    for (auto & it : e->vertices()) {
      if (it == nullptr)
        return false;
    }
    std::pair<EdgeSet::iterator, bool> result = _edges.insert(e);
    if (! result.second)
      return false;
    for (auto it = e->vertices().begin(); it != e->vertices().end(); ++it) {
      Vertex* v = *it;
      v->edges().insert(e);
    }
    return true;
  }

  bool HyperGraph::setEdgeVertex(HyperGraph::Edge* e, int pos, HyperGraph::Vertex* v)
  {
    Vertex* vOld = e->vertex(pos);
    if (vOld != nullptr)
      vOld->edges().erase(e);
    e->setVertex(pos, v);
    if (v != nullptr)
      v->edges().insert(e);
    return true;
  }

  bool HyperGraph::mergeVertices(Vertex* vBig, Vertex* vSmall, bool erase)
  {
    auto it=_vertices.find(vBig->id());
    if (it==_vertices.end())
      return false;

    it=_vertices.find(vSmall->id());
    if (it==_vertices.end())
      return false;

    EdgeSet tmp(vSmall->edges());
    bool ok = true;
    for(auto e : tmp){
      for (size_t i=0; i<e->vertices().size(); i++){
        Vertex* v=e->vertex(i);
        if (v==vSmall)
          ok &= static_cast<int>(setEdgeVertex(e,i,vBig));
      }
    }
    if (erase)
      removeVertex(vSmall);
    return ok;
  }

  bool HyperGraph::detachVertex(Vertex* v){
    auto it=_vertices.find(v->id());
    if (it==_vertices.end())
      return false;
    assert(it->second==v);
    EdgeSet tmp(v->edges());
    for (auto e : tmp){
      for (size_t i = 0 ; i<e->vertices().size(); i++){
	if (v == e->vertex(i))
	  setEdgeVertex(e,i,nullptr);
      }
    }
    return true;
  }

  bool HyperGraph::removeVertex(Vertex* v, bool detach)
  {
    if (detach){
      bool result = detachVertex(v);
      if (! result) {
	assert (0 && "inconsistency in detaching vertex, ");
      }
    }
    auto it=_vertices.find(v->id());
    if (it==_vertices.end())
      return false;
    assert(it->second==v);
    //remove all edges which are entering or leaving v;
    EdgeSet tmp(v->edges());
    for (auto it : tmp){
      if (!removeEdge(it)){
        assert(0 && "error in erasing vertex");
      }
    }
    _vertices.erase(it);
    delete v;
    return true;
  }

  bool HyperGraph::removeEdge(Edge* e)
  {
    auto it = _edges.find(e);
    if (it == _edges.end())
      return false;
    _edges.erase(it);
    for (auto vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
      Vertex* v = *vit;
      if (v == nullptr)
	continue;
      it = v->edges().find(e);
      assert(it!=v->edges().end());
      v->edges().erase(it);
    }

    delete e;
    return true;
  }

  HyperGraph::HyperGraph()
  {
  }

  void HyperGraph::clear()
  {
    for (auto & _vertice : _vertices)
      delete (_vertice.second);
    for (auto _edge : _edges)
      delete _edge;
    _vertices.clear();
    _edges.clear();
  }

  HyperGraph::~HyperGraph()
  {
    clear();
  }

} // namespace g2o
