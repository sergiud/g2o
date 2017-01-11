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

#include "output_helper.h"

#include <cmath>
#include <g2o/core/hyper_graph_action.h>

#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/string_tools.h>

#include <Eigen/Core>

#include <fstream>
#include <iostream>
using namespace std;

G2O_START_NAMESPACE

bool edgeAllVertsSameDim(OptimizableGraph::Edge* e, int dim)
{
  for (auto & i : e->vertices()) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(i);
    if (v->dimension() != dim)
      return false;
  }
  return true;
}

bool saveGnuplot(const std::string& gnudump, const OptimizableGraph& optimizer)
{
  HyperGraph::VertexSet vset;
  for (const auto & it : optimizer.vertices()){
    vset.insert(it.second);
  }
  return saveGnuplot(gnudump, vset, optimizer.edges());
}

bool saveGnuplot(const std::string& gnudump, const HyperGraph::VertexSet& vertices, const HyperGraph::EdgeSet& edges)
{
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot = HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (saveGnuplot == nullptr ){
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered" << endl;
    return false;
  }
  WriteGnuplotAction::Parameters params{};

  int maxDim = -1;
  int minDim = numeric_limits<int>::max();
  for (auto vertice : vertices){
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(vertice);
    int vdim = v->dimension();
    maxDim = (std::max)(vdim, maxDim);
    minDim = (std::min)(vdim, minDim);
  }

  string extension = getFileExtension(gnudump);
  if (extension.empty())
    extension = "dat";
  string baseFilename = getPureFilename(gnudump);

  // check for odometry edges
  bool hasOdomEdge = false;
  bool hasLandmarkEdge = false;
  for (auto edge : edges) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
    if (e->vertices().size() == 2) {
      if (edgeAllVertsSameDim(e, maxDim))
        hasOdomEdge = true;
      else
        hasLandmarkEdge = true;
    }
    if (hasOdomEdge && hasLandmarkEdge)
      break;
  }

  bool fileStatus = true;
  if (hasOdomEdge) {
    string odomFilename = baseFilename + "_odom_edges." + extension;
    cerr << "# saving " << odomFilename << " ... ";
    ofstream fout(odomFilename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing odometry edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      if (e->vertices().size() != 2 || ! edgeAllVertsSameDim(e, maxDim))
        continue;
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (hasLandmarkEdge) {
    string filename = baseFilename + "_landmarks_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing landmark edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      if (e->vertices().size() != 2 || edgeAllVertsSameDim(e, maxDim))
        continue;
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (1) {
    string filename = baseFilename + "_edges." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    // writing all edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      (*saveGnuplot)(e, &params);
    }
    cerr << "done." << endl;
  }

  if (1) {
    string filename = baseFilename + "_vertices." + extension;
    cerr << "# saving " << filename << " ... ";
    ofstream fout(filename.c_str());
    if (! fout) {
      cerr << "Unable to open file" << endl;
      return false;
    }
    params.os = &fout;

    for (auto vertice : vertices){
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(vertice);
      (*saveGnuplot)(v, &params);
    }
    cerr << "done." << endl;
  }

  return fileStatus;
}

bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer)
{
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot = HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (saveGnuplot == nullptr ){
    cerr << __PRETTY_FUNCTION__ << ": no action \"writeGnuplot\" registered" << endl;
    return false;
  }
  WriteGnuplotAction::Parameters params{};
  params.os = &os;

  // writing all edges
  os << "set terminal x11 noraise" << endl;
  os << "set size ratio -1" << endl;
  os << "plot \"-\" w l" << endl;
  for (auto it : optimizer.edges()) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
    (*saveGnuplot)(e, &params);
  }
  os << "e" << endl;

  return true;
}

G2O_END_NAMESPACE
