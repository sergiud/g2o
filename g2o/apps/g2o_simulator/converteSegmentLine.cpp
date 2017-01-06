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

#include <signal.h>
#include <iostream>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cassert>

#include "dl_wrapper.h"
#include "output_helper.h"
#include "g2o_common.h"

#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

#include <g2o/stuff/macros.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/stuff/timeutil.h>
#include <g2o/types/slam2d/types_slam2d.h>

static bool hasToStop=false;

using namespace std;
using namespace g2o;

// sort according to max id, dimension
struct IncrementalEdgesCompare {
  bool operator()(SparseOptimizer::Edge* const & e1, SparseOptimizer::Edge* const & e2)
  {
    const SparseOptimizer::Vertex* to1 = static_cast<const SparseOptimizer::Vertex*>(e1->vertices()[1]);
    const SparseOptimizer::Vertex* to2 = static_cast<const SparseOptimizer::Vertex*>(e2->vertices()[1]);

    int i11 = e1->vertices()[0]->id(), i12 = e1->vertices()[1]->id();
    if (i11 > i12){
      swap(i11, i12);
    }
    int i21 = e2->vertices()[0]->id(), i22 = e2->vertices()[1]->id();
    if (i21 > i22){
      swap(i21, i22);
    }
    if (i12 < i22)
      return true;
    if (i12 > i22)
      return false;
    // push the odometry to be the first
    return to1->dimension() > to2->dimension();
  }
};

void sigquit_handler(int sig)
{
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      cerr << __PRETTY_FUNCTION__ << " forcing exit" << endl;
      exit(1);
    }
  }
}

struct LineInfo {
  LineInfo(VertexSegment2D* s) {
    line=new VertexLine2D();
    line->setId(s->id());
    line->setEstimate(computeLineParameters(s->estimateP1(), s->estimateP2()));
    p1=0;
    p2=0;
  }
  VertexLine2D* line;
  VertexPointXY* p1;
  VertexPointXY* p2;
};

typedef std::map<int, LineInfo> LineInfoMap;

int main(int argc, char** argv)
{
  string outputfilename;
  CommandArgs arg;
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.paramLeftOver("graph-input", inputFilename, "", "graph file which will be processed", true);
  

  arg.parseArgs(argc, argv);

  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);

  if (inputFilename.size() == 0) {
    cerr << "No input data specified" << endl;
    return 0;
  } else if (inputFilename == "-") {
    cerr << "Read input from stdin" << endl;
    if (!optimizer.load(cin)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  } else {
    cerr << "Read input from " << inputFilename << endl;
    ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      cerr << "Failed to open file" << endl;
      return 1;
    }
    if (!optimizer.load(ifs)) {
      cerr << "Error loading graph" << endl;
      return 2;
    }
  }
  cerr << "Loaded " << optimizer.vertices().size() << " vertices" << endl;
  cerr << "Loaded " << optimizer.edges().size() << " edges" << endl;

  LineInfoMap lmap;
  OptimizableGraph outGraph();
  // insert all lines in the infomap
  int currentId = 0;
  for (OptimizableGraph::VertexIDMap::iterator it=optimizer.vertices().begin(); it!=optimizer->vertices().end(); it++){
    VertexSE2 *p=dynamic_cast<VertexSegment2D*> (it->second());
    if (p){
      VertexSE2 *np=p->clone();
      np->setId(p->id());
      outGraph->addVertex(np);
    }
    VertexSegment2D* s=dynamic_cast<VertexSegment2D*> (it->second());
    if (s){
      lmap.insert(make_pair(s->id(), LineInfo(s)));
    }
    currentId = it->first;
  }
  currentId++;
  for (OptimizableGraph::EdgeSet::iterator it=optimizer.edges().begin(); it!=optimizer->edges().end(); it++){
    EdgeSE2* ods=dynamic_cast<EdgeSE2*> (*it);
    if (ods){
      EdgeSE2* ods2=new EdgeSE2();
      ods2->setMeasdurement(ods2->measurement());
      ods2->setInformation(ods2->information());
      ods2->vertices[0]=outGraph->vertices(ods->vertices()[0]->id());
      ods2->vertices[1]=outGraph->vertices(ods->vertices()[1]->id());
      outGraph->addEdge(ods2);
    }

    EdgeSE2Segment2D* es=dynamic_cast<EdgeSE2Segment2D*> (*it);
    EdgeSE2Segment2DLine* el=dynamic_cast<EdgeSE2Segment2DLine*> (*it);
    EdgeSE2Segment2DPointLine* espl=dynamic_cast<EdgeSE2Segment2DPointLine*> (*it);

    if (es || el || espl){
      VertexSE2* pose = dynamic_cast<VertexSE2*>((*it)->vertices()[0]);
      VertexSegment2D* segment = dynamic_cast<VertexSegment2D*>((*it)->vertices()[1]);
      if (!pose)
        continue;
      pose=dynamic_cast<VertexSE2*>(outGraph->vertex(pose->id()));
      LineInfo& linfo = lmap[(*it)->vertices()[0]->id()];
      VertexLine* line =linfo->line;

      EdgeSE2Line el2=new EdgeSE2Line();
      VertexPoint* & p1=linfo->p1;
      VertexPoint* & p2=linfo->p2;
      el2->vertices()[0]=p;
      el2->vertices()[1]=l;
      if (el) {
        el2->setMeasurement(el->measurement());
        el2->setInformation(el->information());
        outGraph->addEdge(el2);
      }
      if (es) {
        el2-setMeasurement(computeLineParameters(s->measurementP1(), s->measurementP2()));
        if (!p1){
          p1=new VertexPointXY();
          p1->setEstimate(segment->estimateP1());
          p1->setId(currentId++);
          outGraph->addVertex(p1);
          
          EdgeLine2DPointXY* p1e=new EdgeLine2DPointXY();
          p1e->vertices()[0]=line;
          p1e->vertices()[1]=p1;
          p1e->setMeasurement(0);
          Matrix1d p1i;
          p1i(0,0)=1e6;
          p1e->setInformation(p1i);
          outGraph->addEdge(p1);
        }
        if (! p2) {
          p2=new VertexPointXY();
          p2->setEstimate(segment->estimateP2());
          p2->setId(currentId++);
          outGraph->addVertex(p2);

          EdgeLine2DPointXY* p2e=new EdgeLine2DPointXY();
          p2e->vertices()[0]=line;
          p2e->vertices()[1]=p2;
          p2e->setMeasurement(0);
          Matrix1d p2i;
          p2i(0,0)=1e6;
          p2e->setInformation(p2i);
          outGraph->addEdge(p2);
        }
        EdgeSE2PointXY* p1e = new EdgeSE2PointXY();
        p1e->vertices()[0]=line;
        p1e->vertices()[1]=p1;
        p1e->setMeasurement(es->measurementP1());
        Matrix4d si=segment->information();
        Matrix2d p1i=si.block<2,2>(0,0);
        p1e->setInformation(p1i);
        outGraph->addEdge(p1e);

        EdgeSE2PointXY* p2e = new EdgeSE2PointXY();
        p2e->vertices()[0]=line;
        p2e->vertices()[1]=p2;
        p2e->setMeasurement(es->measurementP2());
        Matrix4d si=segment->information();
        Matrix2d p2i=si.block<2,2>(2,2);
        p2e->setInformation(p2i);
        outGraph->addEdge(p2e);
      }

      if (espl) {
        Vector2d lparams;
        lparams[0]=espl->theta();
        Vector2d n(cos(espl->theta()), sin(espl->theta()));
        lparams[1]=n.dot(espl->point());
        Matrix2d li;
        li << 1000, 0, 0, 100;
        el2->setMeasurement(lparams);
        el2->setInformation(li);

        VertexPointXY*& pX = (espl->pointNum()==0 )? p1:p2;

        if (!pX){
          pX=new VertexPointXY();
          pX->setEstimate(segment->estimatePX());
          pX->setId(currentId++);
          outGraph->addVertex(pX);
          
          EdgeLine2DPointXY* pXe=new EdgeLine2DPointXY();
          pXe->vertices()[0]=line;
          pXe->vertices()[1]=pX;
          pXe->setMeasurement(0);
          Matrix1d pXi;
          pXi(0,0)=1e6;
          pXe->setInformation(pXi);
          outGraph->addEdge(pX);
        }

        EdgeSE2PointXY* pXe = new EdgeSE2PointXY();
        pXe->vertices()[0]=line;
        pXe->vertices()[1]=pX;
        pXe->setMeasurement(es->measurementPX());
        Matrix4d si=segment->information();
        Matrix2d pXi=si.block<2,2>(0,0);
        pXe->setInformation(pXi);
        outGraph->addEdge(pXe);
      }
    }
  }

  
  if (outputfilename.size() > 0) {
    if (outputfilename == "-") {
      cerr << "saving to stdout";
      optimizer.save(cout);
    } else {
      cerr << "saving " << outputfilename << " ... ";
      optimizer.save(outputfilename.c_str());
    }
    cerr << "done." << endl;
  }

  // destroy all the singletons
  //Factory::destroy();
  //OptimizationAlgorithmFactory::destroy();
  //HyperGraphActionLibrary::destroy();

  return 0;
}
