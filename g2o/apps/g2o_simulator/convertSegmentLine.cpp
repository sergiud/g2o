// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/apps/g2o_simulator/simutils.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/timeutil.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>

#include <algorithm>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace std;
using namespace g2o;
using namespace Eigen;

//! The vertices that replace a segment, which the output graph owns.
struct LineInfo {
  explicit LineInfo(VertexLine2D* line_) : line(line_) {
    p1 = nullptr;
    p2 = nullptr;
  }
  VertexLine2D* line;
  VertexPointXY* p1;
  VertexPointXY* p2;
};

typedef std::map<int, LineInfo> LineInfoMap;

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  string outputfilename;
  string inputFilename;
  CommandArgs arg;
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed", true);

  arg.parseArgs(argc, argv);
  OptimizableGraph inGraph;

  // registering all the types from the libraries

  if (inputFilename.empty()) {
    ngx::Log(ngx::Error, "No input data specified");
    return 0;
  } else if (inputFilename == "-") {
    ngx::Log("Read input from stdin");
    if (!inGraph.load(cin)) {
      ngx::Log(ngx::Error, "Error loading graph");
      return 2;
    }
  } else {
    ngx::Log("Read input from {}", inputFilename);
    ifstream ifs(inputFilename.c_str());
    if (!ifs) {
      ngx::Log(ngx::Error, "Failed to open file");
      return 1;
    }
    if (!inGraph.load(ifs)) {
      ngx::Log(ngx::Error, "Error loading graph");
      return 2;
    }
  }
  ngx::Log("Loaded {} vertices", inGraph.vertices().size());
  ngx::Log("Loaded {} edges", inGraph.edges().size());

  ngx::Log("filling in linfoMap and odoms");
  LineInfoMap lmap;
  OptimizableGraph outGraph;
  // insert all lines in the infomap
  int currentId = -1000;
  bool firstVertexFound = false;
  for (auto& it : inGraph.vertices()) {
    currentId = currentId > it.first ? currentId : it.first;

    VertexSE2* pose = dynamic_cast<VertexSE2*>(it.second.get());
    if (pose != nullptr) {
      auto nposeOwner = std::make_unique<VertexSE2>();
      auto* npose = nposeOwner.get();
      npose->setEstimate(pose->estimate());
      npose->setId(pose->id());
      outGraph.addVertex(std::move(nposeOwner));
      if (!firstVertexFound) {
        firstVertexFound = true;
        npose->setFixed(true);
      }
    }

    VertexSegment2D* s = dynamic_cast<VertexSegment2D*>(it.second.get());
    if (s != nullptr) {
      auto line = std::make_unique<VertexLine2D>();
      line->setId(s->id());
      line->setEstimate(
          computeLineParameters(s->estimateP1(), s->estimateP2()));
      lmap.insert(make_pair(s->id(), LineInfo(line.get())));
      outGraph.addVertex(std::move(line));
    }
  }

  currentId++;

  ngx::Log("filling in edges and odoms");
  for (const auto& it : inGraph.edges()) {
    EdgeSE2* ods = dynamic_cast<EdgeSE2*>(it.get());
    if (ods != nullptr) {
      auto ods2 = std::make_unique<EdgeSE2>();
      ods2->setMeasurement(ods->measurement());
      ods2->setInformation(ods->information());
      ods2->vertices()[0] = outGraph.vertex(ods->vertices()[0]->id());
      ods2->vertices()[1] = outGraph.vertex(ods->vertices()[1]->id());
      outGraph.addEdge(std::move(ods2));
    }

    EdgeSE2Segment2D* es = dynamic_cast<EdgeSE2Segment2D*>(it.get());
    EdgeSE2Segment2DLine* el = dynamic_cast<EdgeSE2Segment2DLine*>(it.get());
    EdgeSE2Segment2DPointLine* espl =
        dynamic_cast<EdgeSE2Segment2DPointLine*>(it.get());

    if ((es != nullptr) || (el != nullptr) || (espl != nullptr)) {
      VertexSE2* pose = dynamic_cast<VertexSE2*>(it->vertices()[0]);
      VertexSegment2D* segment =
          dynamic_cast<VertexSegment2D*>(it->vertices()[1]);
      if (pose == nullptr) continue;
      pose = dynamic_cast<VertexSE2*>(outGraph.vertex(pose->id()));
      auto lit = lmap.find(segment->id());
      assert(lit != lmap.end());
      LineInfo& linfo = lit->second;
      VertexLine2D* line = linfo.line;
      VertexPointXY*& p1 = linfo.p1;
      VertexPointXY*& p2 = linfo.p2;

      auto el2 = std::make_unique<EdgeSE2Line2D>();
      el2->vertices()[0] = pose;
      el2->vertices()[1] = line;
      if (el != nullptr) {
        el2->setMeasurement(el->measurement());
        el2->setInformation(el->information());
        outGraph.addEdge(std::move(el2));
      } else if (es != nullptr) {
        el2->setMeasurement(
            computeLineParameters(es->measurementP1(), es->measurementP2()));
        Matrix2d el2info;
        el2info << 10000, 0, 0, 1000;
        el2->setInformation(el2info);
        outGraph.addEdge(std::move(el2));
        Matrix4d si = es->information();
        if (p1 == nullptr) {
          auto p1Owner = std::make_unique<VertexPointXY>();
          p1 = p1Owner.get();
          p1->setEstimate(segment->estimateP1());
          p1->setId(currentId++);
          outGraph.addVertex(std::move(p1Owner));
          line->p1Id = p1->id();

          auto p1e = std::make_unique<EdgeLine2DPointXY>();
          p1e->vertices()[0] = line;
          p1e->vertices()[1] = p1;
          p1e->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> p1i;
          p1i(0, 0) = 1e6;
          p1e->setInformation(p1i);
          outGraph.addEdge(std::move(p1e));
        }
        if (p2 == nullptr) {
          auto p2Owner = std::make_unique<VertexPointXY>();
          p2 = p2Owner.get();
          p2->setEstimate(segment->estimateP2());
          p2->setId(currentId++);
          outGraph.addVertex(std::move(p2Owner));
          line->p2Id = p2->id();

          auto p2e = std::make_unique<EdgeLine2DPointXY>();
          p2e->vertices()[0] = line;
          p2e->vertices()[1] = p2;
          p2e->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> p2i;
          p2i(0, 0) = 1e6;
          p2e->setInformation(p2i);
          outGraph.addEdge(std::move(p2e));
        }

        auto p1e = std::make_unique<EdgeSE2PointXY>();
        p1e->vertices()[0] = pose;
        p1e->vertices()[1] = p1;
        p1e->setMeasurement(es->measurementP1());
        Matrix2d p1i = si.block<2, 2>(0, 0);
        p1e->setInformation(p1i);
        outGraph.addEdge(std::move(p1e));

        auto p2e = std::make_unique<EdgeSE2PointXY>();
        p2e->vertices()[0] = pose;
        p2e->vertices()[1] = p2;
        p2e->setMeasurement(es->measurementP2());
        Matrix2d p2i = si.block<2, 2>(2, 2);
        p2e->setInformation(p2i);
        outGraph.addEdge(std::move(p2e));
      } else if (espl != nullptr) {
        Matrix3d si = espl->information();
        Vector2d lparams;
        lparams[0] = espl->theta();
        Vector2d n(cos(espl->theta()), sin(espl->theta()));
        lparams[1] = n.dot(espl->point());
        Matrix2d li;
        li << si(2, 2), 0, 0, 1000;
        el2->setMeasurement(lparams);
        el2->setInformation(li);
        outGraph.addEdge(std::move(el2));

        VertexPointXY*& pX = (espl->pointNum() == 0) ? p1 : p2;
        if (pX == nullptr) {
          ngx::Log("mkp: {}", line->id());
          auto pXOwner = std::make_unique<VertexPointXY>();
          pX = pXOwner.get();
          pX->setId(currentId++);
          outGraph.addVertex(std::move(pXOwner));

          Vector2d estPx;
          if (espl->pointNum() == 0) {
            estPx = segment->estimateP1();
            line->p1Id = pX->id();
          } else {
            estPx = segment->estimateP2();
            line->p2Id = pX->id();
          }
          pX->setEstimate(estPx);

          auto pXe = std::make_unique<EdgeLine2DPointXY>();
          pXe->vertices()[0] = line;
          pXe->vertices()[1] = pX;
          pXe->setMeasurement(0);
          Eigen::Matrix<double, 1, 1> pXi;
          pXi(0, 0) = 1e6;
          pXe->setInformation(pXi);
          outGraph.addEdge(std::move(pXe));
        }

        auto pXe = std::make_unique<EdgeSE2PointXY>();
        pXe->vertices()[0] = pose;
        pXe->vertices()[1] = pX;
        pXe->setMeasurement(espl->point());
        Matrix2d pXi = si.block<2, 2>(0, 0);
        pXe->setInformation(pXi);
        outGraph.addEdge(std::move(pXe));
      }
    }
  }

  if (!outputfilename.empty()) {
    if (outputfilename == "-") {
      ngx::Log("saving to stdout");
      outGraph.save(cout);
    } else {
      ngx::Log("saving {}", outputfilename);
      outGraph.save(outputfilename.c_str());
    }
    ngx::Log("done.");
  }

  // destroy all the singletons
  // Factory::destroy();
  // OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();

  return 0;
}
