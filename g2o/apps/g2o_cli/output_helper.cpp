// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "output_helper.h"

#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/string_tools.h>

#include <Eigen/Core>
#include <cmath>
#include <fstream>
#include <iostream>
using namespace std;

G2O_START_NAMESPACE

bool edgeAllVertsSameDim(OptimizableGraph::Edge* e, int dim) {
  for (auto& i : e->vertices()) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(i);
    if (v->dimension() != dim) return false;
  }
  return true;
}

bool saveGnuplot(const std::string& gnudump,
                 const OptimizableGraph& optimizer) {
  HyperGraph::VertexSet vset;
  for (const auto& it : optimizer.vertices()) {
    vset.insert(it.second);
  }
  return saveGnuplot(gnudump, vset, optimizer.edges());
}

bool saveGnuplot(const std::string& gnudump,
                 const HyperGraph::VertexSet& vertices,
                 const HyperGraph::EdgeSet& edges) {
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot =
      HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (saveGnuplot == nullptr) {
    ngx::Log(ngx::Error, "no action \"writeGnuplot\" registered");
    return false;
  }
  WriteGnuplotAction::Parameters params{};

  int maxDim = -1;
  int minDim = numeric_limits<int>::max();
  for (auto vertice : vertices) {
    OptimizableGraph::Vertex* v =
        dynamic_cast<OptimizableGraph::Vertex*>(vertice);
    int vdim = v->dimension();
    maxDim = (std::max)(vdim, maxDim);
    minDim = (std::min)(vdim, minDim);
  }

  string extension = getFileExtension(gnudump);
  if (extension.empty()) extension = "dat";
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
    if (hasOdomEdge && hasLandmarkEdge) break;
  }

  bool fileStatus = true;
  if (hasOdomEdge) {
    string odomFilename = baseFilename + "_odom_edges." + extension;
    ngx::Log("# saving {}", odomFilename);
    ofstream fout(odomFilename.c_str());
    if (!fout) {
      ngx::Log(ngx::Error, "Unable to open file");
      return false;
    }
    params.os = &fout;

    // writing odometry edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      if (e->vertices().size() != 2 || !edgeAllVertsSameDim(e, maxDim))
        continue;
      (*saveGnuplot)(e, &params);
    }
    ngx::Log("done.");
  }

  if (hasLandmarkEdge) {
    string filename = baseFilename + "_landmarks_edges." + extension;
    ngx::Log("# saving {}", filename);
    ofstream fout(filename.c_str());
    if (!fout) {
      ngx::Log(ngx::Error, "Unable to open file");
      return false;
    }
    params.os = &fout;

    // writing landmark edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      if (e->vertices().size() != 2 || edgeAllVertsSameDim(e, maxDim)) continue;
      (*saveGnuplot)(e, &params);
    }
    ngx::Log("done.");
  }

  if (1) {
    string filename = baseFilename + "_edges." + extension;
    ngx::Log("# saving {}", filename);
    ofstream fout(filename.c_str());
    if (!fout) {
      ngx::Log(ngx::Error, "Unable to open file");
      return false;
    }
    params.os = &fout;

    // writing all edges
    for (auto edge : edges) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(edge);
      (*saveGnuplot)(e, &params);
    }
    ngx::Log("done.");
  }

  if (1) {
    string filename = baseFilename + "_vertices." + extension;
    ngx::Log("# saving {}", filename);
    ofstream fout(filename.c_str());
    if (!fout) {
      ngx::Log(ngx::Error, "Unable to open file");
      return false;
    }
    params.os = &fout;

    for (auto vertice : vertices) {
      OptimizableGraph::Vertex* v =
          dynamic_cast<OptimizableGraph::Vertex*>(vertice);
      (*saveGnuplot)(v, &params);
    }
    ngx::Log("done.");
  }

  return fileStatus;
}

bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer) {
  // seek for an action whose name is writeGnuplot in the library
  HyperGraphElementAction* saveGnuplot =
      HyperGraphActionLibrary::instance()->actionByName("writeGnuplot");
  if (saveGnuplot == nullptr) {
    ngx::Log(ngx::Error, "no action \"writeGnuplot\" registered");
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
