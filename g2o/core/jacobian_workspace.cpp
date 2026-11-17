// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/jacobian_workspace.h>

#include <cmath>

#include <g2o/core/optimizable_graph.h>

using namespace std;

G2O_START_NAMESPACE

JacobianWorkspace::JacobianWorkspace() :
  _maxNumVertices(-1), _maxDimension(-1)
{
}

JacobianWorkspace::~JacobianWorkspace()
= default;

bool JacobianWorkspace::allocate()
{
  //cerr << __PRETTY_FUNCTION__ << " " << PVAR(this) << " " << PVAR(_maxNumVertices) << " " << PVAR(_maxDimension) << endl;
  if (_maxNumVertices <=0 || _maxDimension <= 0)
    return false;
  _workspace.resize(_maxNumVertices);
  for (auto & it : _workspace) {
    it.resize(_maxDimension);
    it.setZero();
  }
  return true;
}

void JacobianWorkspace::updateSize(const HyperGraph::Edge* e_)
{
  const OptimizableGraph::Edge* e = dynamic_cast<const OptimizableGraph::Edge*>(e_);
  int errorDimension = e->dimension();
  int numVertices = e->vertices().size();
  int maxDimensionForEdge = -1;
  for (int i = 0; i < numVertices; ++i) {
    const OptimizableGraph::Vertex* v = dynamic_cast<const OptimizableGraph::Vertex*>(e->vertex(i));
    assert(v && "Edge has no vertex assigned");
    maxDimensionForEdge = max(v->dimension() * errorDimension, maxDimensionForEdge);
  }
  _maxNumVertices = max(numVertices, _maxNumVertices);
  _maxDimension = max(maxDimensionForEdge, _maxDimension);
  //cerr << __PRETTY_FUNCTION__ << " " << PVAR(this) << " " << PVAR(_maxNumVertices) << " " << PVAR(_maxDimension) << endl;
}

void JacobianWorkspace::updateSize(const OptimizableGraph& graph)
{
  for (auto it : graph.edges()) {
    const OptimizableGraph::Edge* e = dynamic_cast<const OptimizableGraph::Edge*>(it);
    updateSize(e);
  }
}

void JacobianWorkspace::updateSize(int numVertices, int dimension)
{
  _maxNumVertices = max(numVertices, _maxNumVertices);
  _maxDimension = max(dimension, _maxDimension);
}

G2O_END_NAMESPACE
