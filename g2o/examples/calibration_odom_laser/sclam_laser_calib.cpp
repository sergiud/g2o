// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/interrupt.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/streamed.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/stuff/timeutil.h>
#include <g2o/types/data/types_data.h>
#include <g2o/types/sclam2d/types_sclam2d.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>

#include "gm2dl_io.h"

using namespace std;
using namespace g2o;

G2O_USE_OPTIMIZATION_LIBRARY(csparse);
G2O_USE_TYPE_GROUP(slam2d);

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  int maxIterations;
  bool verbose;
  string inputFilename;
  string gnudump;
  string outputfilename;
  bool initialGuess;
  // command line parsing
  CommandArgs commandLineArguments;
  commandLineArguments.param("i", maxIterations, 10, "perform n iterations");
  commandLineArguments.param("v", verbose, false,
                             "verbose output of the optimization process");
  commandLineArguments.param("guess", initialGuess, false,
                             "initial guess based on spanning tree");
  commandLineArguments.param("gnudump", gnudump, "",
                             "dump to gnuplot data file");
  commandLineArguments.param("o", outputfilename, "",
                             "output final version of the graph");
  commandLineArguments.paramLeftOver("gm2dl-input", inputFilename, "",
                                     "gm2dl file which will be processed");

  commandLineArguments.parseArgs(argc, argv);

  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);

  OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(solverFactory->construct("lm_var", solverProperty));

  // loading
  if (!Gm2dlIO::readGm2dl(inputFilename, optimizer, false)) {
    ngx::Log(ngx::Error, "Error while loading gm2dl file");
    return EXIT_FAILURE;
  }

  VertexSE2* laserOffset =
      dynamic_cast<VertexSE2*>(optimizer.vertex(numeric_limits<int>::max()));
  // laserOffset->setEstimate(SE2()); // set to Identity
  if (laserOffset != nullptr) {
    ngx::Log("Initial laser offset {}",
             Streamed(laserOffset->estimate().toVector().transpose()));
  }
  bool gaugeFreedom = optimizer.gaugeFreedom();

  OptimizableGraph::Vertex* gauge = optimizer.findGauge();
  if (gaugeFreedom) {
    if (gauge == nullptr) {
      ngx::Log(ngx::Error, "# cannot find a vertex to fix in this thing");
      return 2;
    } else {
      ngx::Log("# graph is fixed by node {}", gauge->id());
      gauge->setFixed(true);
    }
  } else {
    ngx::Log("# graph is fixed by priors");
  }

  // sanity check
  HyperDijkstra d(&optimizer);
  UniformCostFunction f;
  d.shortestPaths(gauge, &f);
  // cerr << PVAR(d.visited().size()) << endl;

  if (d.visited().size() != optimizer.vertices().size()) {
    ngx::Log(ngx::Warning, "d.visited().size() != optimizer.vertices().size()");
    ngx::Log("visited: {}", d.visited().size());
    ngx::Log("vertices: {}", optimizer.vertices().size());
    if (1)
      for (SparseOptimizer::VertexIDMap::const_iterator it =
               optimizer.vertices().begin();
           it != optimizer.vertices().end(); ++it) {
        OptimizableGraph::Vertex* v =
            dynamic_cast<OptimizableGraph::Vertex*>(it->second.get());
        if (d.visited().count(v) == 0) {
          ngx::Log("\t unvisited vertex {} {}", it->first, (void*)v);
          v->setFixed(true);
        }
      }
  }

  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  ngx::Log("Initial chi2 = {:f}", optimizer.chi2());

  // if (guessCostFunction)
  // optimizer.computeInitialGuess(guessCostFunction);

  // Interrupts stop the optimization from here on.
  optimizer.setForceStopFlag(&stopOnInterrupt());
  int i = optimizer.optimize(maxIterations);
  if (maxIterations > 0 && (i == 0)) {
    ngx::Log(ngx::Error, "optimize failed, result might be invalid");
    return EXIT_FAILURE;
  }

  if (laserOffset != nullptr) {
    ngx::Log("Calibrated laser offset {}",
             Streamed(laserOffset->estimate().toVector().transpose()));
  }

  if (!outputfilename.empty()) {
    Gm2dlIO::updateLaserData(optimizer);
    ngx::Log("Writing {}", outputfilename);
    bool writeStatus = Gm2dlIO::writeGm2dl(outputfilename, optimizer);
    if (writeStatus) {
      ngx::Log("done.");
    } else {
      ngx::Log(ngx::Error, "failed");
      return EXIT_FAILURE;
    }
  }

  if (!gnudump.empty()) {
    ofstream fout(gnudump.c_str());
    for (SparseOptimizer::VertexIDMap::const_iterator it =
             optimizer.vertices().begin();
         it != optimizer.vertices().end(); ++it) {
      VertexSE2* v = dynamic_cast<VertexSE2*>(it->second.get());
      fout << v->estimate().toVector().transpose() << endl;
    }
  }

  return EXIT_SUCCESS;
}
