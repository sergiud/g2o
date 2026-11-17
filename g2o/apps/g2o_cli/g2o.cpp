// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/stuff/timeutil.h>
#include <ng-log/experimental/signal_safe_log.h>

#include <algorithm>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "dl_wrapper.h"
#include "g2o_common.h"
#include "output_helper.h"

static bool hasToStop = false;

using namespace std;
using namespace g2o;
using namespace Eigen;

// sort according to max id, dimension
struct IncrementalEdgesCompare {
  bool operator()(SparseOptimizer::Edge* const& e1,
                  SparseOptimizer::Edge* const& e2) {
    const SparseOptimizer::Vertex* to1 =
        dynamic_cast<const SparseOptimizer::Vertex*>(e1->vertices()[1]);
    const SparseOptimizer::Vertex* to2 =
        dynamic_cast<const SparseOptimizer::Vertex*>(e2->vertices()[1]);

    int i11 = e1->vertices()[0]->id(), i12 = e1->vertices()[1]->id();
    if (i11 > i12) {
      swap(i11, i12);
    }
    int i21 = e2->vertices()[0]->id(), i22 = e2->vertices()[1]->id();
    if (i21 > i22) {
      swap(i21, i22);
    }
    if (i12 < i22) return true;
    if (i12 > i22) return false;
    // push the odometry to be the first
    return to1->dimension() > to2->dimension();
  }
};

void sigquit_handler(int sig) {
  if (sig == SIGINT) {
    hasToStop = 1;
    static int cnt = 0;
    if (cnt++ == 2) {
      ngx::Log(ngx::SignalSafe | ngx::Warning, "forcing exit");
      exit(1);
    }
  }
}

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  OptimizableGraph::initMultiThreading();
  int maxIterations;
  bool verbose;
  string inputFilename;
  string gnudump;
  string outputfilename;
  string solverProperties;
  string strSolver;
  string loadLookup;
  bool initialGuess;
  bool initialGuessOdometry;
  bool marginalize;
  bool listTypes;
  bool listSolvers;
  bool listRobustKernels;
  bool incremental;
  bool guiOut;
  int gaugeId;
  string robustKernel;
  bool computeMarginals;
  bool printSolverProperties;
  double huberWidth;
  double gain;
  int maxIterationsWithGain;
  // double lambdaInit;
  int updateGraphEachN = 10;
  string statsFile;
  string summaryFile;
  bool nonSequential;
  // command line parsing
  std::vector<int> gaugeList;
  CommandArgs arg;
  arg.param("i", maxIterations, 5,
            "perform n iterations, if negative consider the gain");
  arg.param("gain", gain, 1e-6,
            "the gain used to stop optimization (default = 1e-6)");
  arg.param("ig", maxIterationsWithGain, std::numeric_limits<int>::max(),
            "Maximum number of iterations with gain enabled (default: inf)");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("guess", initialGuess, false,
            "initial guess based on spanning tree");
  arg.param("guessOdometry", initialGuessOdometry, false,
            "initial guess based on odometry");
  arg.param("inc", incremental, false, "run incremetally");
  arg.param("update", updateGraphEachN, 10, "updates after x odometry nodes");
  arg.param("guiout", guiOut, false, "gui output while running incrementally");
  arg.param("marginalize", marginalize, false, "on or off");
  arg.param("printSolverProperties", printSolverProperties, false,
            "print the properties of the solver");
  arg.param("solverProperties", solverProperties, "",
            "set the internal properties of a solver,\n\te.g., "
            "initialLambda=0.0001,maxTrialsAfterFailure=2");
  arg.param("gnudump", gnudump, "", "dump to gnuplot data file");
  arg.param("robustKernel", robustKernel, "", "use this robust error function");
  arg.param("robustKernelWidth", huberWidth, -1.,
            "width for the robust Kernel (only if robustKernel)");
  arg.param("computeMarginals", computeMarginals, false,
            "computes the marginal covariances of something. FOR TESTING ONLY");
  arg.param("gaugeId", gaugeId, -1, "force the gauge");
  arg.param("o", outputfilename, "", "output final version of the graph");
  arg.param("solver", strSolver, "gn_var",
            "specify which solver to use underneat\n\t {gn_var, lm_fix3_2, "
            "gn_fix6_3, lm_fix7_3}");
#ifndef G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
  string dummy;
  arg.param("solverlib", dummy, "",
            "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "",
            "specify a types library which will be loaded");
#endif
  arg.param("stats", statsFile, "", "specify a file for the statistics");
  arg.param("listTypes", listTypes, false, "list the registered types");
  arg.param("listRobustKernels", listRobustKernels, false,
            "list the registered robust kernels");
  arg.param("listSolvers", listSolvers, false, "list the available solvers");
  arg.param("renameTypes", loadLookup, "",
            "create a lookup for loading types into other types,\n\t "
            "TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., "
            "VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.param("gaugeList", gaugeList, std::vector<int>(),
            "set the list of gauges separated by commas without spaces \n  "
            "e.g: 1,2,3,4,5 ");
  arg.param("summary", summaryFile, "",
            "append a summary of this optimization run to the summary file "
            "passed as argument");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed", true);
  arg.param("nonSequential", nonSequential, false,
            "apply the robust kernel only on loop closures and not odometries");

  arg.parseArgs(argc, argv);

  if (verbose) {
    ngx::Log("# Used Compiler: {}", G2O_CXX_COMPILER);
  }

#ifndef G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
  // registering all the types from the libraries
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);
  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);
#else
  if (verbose) ngx::Log("# linked version of g2o");
#endif

  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();
  if (listSolvers) {
    solverFactory->listSolvers(cout);
  }

  if (listTypes) {
    Factory::instance()->printRegisteredTypes(cout, true);
  }

  if (listRobustKernels) {
    std::vector<std::string> kernels;
    RobustKernelFactory::instance()->fillKnownKernels(kernels);
    cout << "Robust Kernels:" << endl;
    for (auto& kernel : kernels) {
      cout << kernel << endl;
    }
  }

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);

  SparseOptimizerTerminateAction* terminateAction = nullptr;
  if (maxIterations < 0) {
    ngx::Log(
        "# setup termination criterion based on the gain of the iteration");
    maxIterations = maxIterationsWithGain;
    terminateAction = new SparseOptimizerTerminateAction;
    terminateAction->setGainThreshold(gain);
    terminateAction->setMaxIterations(maxIterationsWithGain);
    optimizer.addPostIterationAction(terminateAction);
  }

  // allocating the desired solver + testing whether the solver is okay
  OptimizationAlgorithmProperty solverProperty;
  optimizer.setAlgorithm(solverFactory->construct(strSolver, solverProperty));
  if (optimizer.solver() == nullptr) {
    ngx::Log(ngx::Error, "Error allocating solver. Allocating \"{}\" failed!",
             strSolver);
    return 0;
  }

  if (!solverProperties.empty()) {
    bool updateStatus =
        optimizer.solver()->updatePropertiesFromString(solverProperties);
    if (!updateStatus) {
      ngx::Log(
          ngx::Error,
          "Failure while updating the solver properties from the given string");
    }
  }
  if (!solverProperties.empty() || printSolverProperties) {
    optimizer.solver()->printProperties(cerr);
  }

  // Loading the input data
  if (!loadLookup.empty()) {
    optimizer.setRenamedTypesFromString(loadLookup);
  }
  if (inputFilename.empty()) {
    ngx::Log(ngx::Error, "No input data specified");
    return 0;
  } else if (inputFilename == "-") {
    ngx::Log("Read input from stdin");
    if (!optimizer.load(cin)) {
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
    if (!optimizer.load(ifs)) {
      ngx::Log(ngx::Error, "Error loading graph");
      return 2;
    }
  }
  ngx::Log("Loaded {} vertices", optimizer.vertices().size());
  ngx::Log("Loaded {} edges", optimizer.edges().size());

  if (optimizer.vertices().empty()) {
    ngx::Log(ngx::Error, "Graph contains no vertices");
    return 1;
  }

  set<int> vertexDimensions = optimizer.dimensions();
  if (!optimizer.isSolverSuitable(solverProperty, vertexDimensions)) {
    ngx::Log(
        ngx::Error,
        "The selected solver is not suitable for optimizing the given graph");
    return 3;
  }
  assert(optimizer.solver());
  // optimizer.setMethod(str2method(strMethod));
  // optimizer.setUserLambdaInit(lambdaInit);

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = optimizer.gaugeFreedom();
  OptimizableGraph::Vertex* gauge = nullptr;
  if (!gaugeList.empty() != 0u) {
    std::string fixedGauges;
    for (size_t i = 0; i < gaugeList.size(); i++) {
      int id = gaugeList[i];
      OptimizableGraph::Vertex* v = optimizer.vertex(id);
      if (v == nullptr) {
        ngx::Log(ngx::Error,
                 "not found the vertex of id {} in the gaugeList. Aborting",
                 id);
        return -1;
      } else {
        if (i == 0) gauge = v;
        std::format_to(std::back_inserter(fixedGauges), " {}", v->id());
        v->setFixed(1);
      }
    }
    ngx::Log("Fixing gauges:{}", fixedGauges);
    gaugeFreedom = false;
  } else {
    gauge = optimizer.findGauge();
  }
  if (gaugeFreedom) {
    if (gauge == nullptr) {
      ngx::Log(ngx::Error, "# cannot find a vertex to fix in this thing");
      return 2;
    } else {
      ngx::Log("# graph is fixed by node {}", gauge->id());
      gauge->setFixed(true);
    }
  } else {
    ngx::Log("# graph is fixed by priors or already fixed vertex");
  }

  // if schur, we wanna marginalize the landmarks...
  if (marginalize || solverProperty.requiresMarginalize) {
    int maxDim = *vertexDimensions.rbegin();
    int minDim = *vertexDimensions.begin();
    if (maxDim != minDim) {
      ngx::Log("# Preparing Marginalization of the Landmarks");
      for (auto& it : optimizer.vertices()) {
        OptimizableGraph::Vertex* v =
            dynamic_cast<OptimizableGraph::Vertex*>(it.second);
        if (v->dimension() != maxDim) {
          v->setMarginalized(true);
        }
      }
      ngx::Log("done.");
    }
  }

  if (!robustKernel.empty()) {
    AbstractRobustKernelCreator* creator =
        RobustKernelFactory::instance()->creator(robustKernel);
    ngx::Log("# Preparing robust error function");
    if (creator != nullptr) {
      if (nonSequential) {
        for (auto it : optimizer.edges()) {
          SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(it);
          if (e->vertices().size() >= 2 &&
              std::abs(e->vertex(0)->id() - e->vertex(1)->id()) != 1) {
            e->setRobustKernel(creator->construct());
            if (huberWidth > 0) e->robustKernel()->setDelta(huberWidth);
          }
        }
      } else {
        for (auto it : optimizer.edges()) {
          SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(it);
          e->setRobustKernel(creator->construct());
          if (huberWidth > 0) e->robustKernel()->setDelta(huberWidth);
        }
      }
      ngx::Log("done.");
    } else {
      ngx::Log(ngx::Error, "Unknown Robust Kernel: {}", robustKernel);
    }
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
  }

  if (incremental) {
    ngx::Log("# Note: this variant performs batch steps in each time step");
    ngx::Log(
        "#       For a variant which updates the Cholesky factor use the "
        "binary g2o_incremental");
    int incIterations = maxIterations;
    if (!arg.parsedParam("i")) {
      ngx::Log("# Setting default number of iterations");
      incIterations = 1;
    }
    int updateDisplayEveryN = updateGraphEachN;
    int maxDim = 0;

    ngx::Log("# incremental settings");
    ngx::Log("#\t solve every {}", updateGraphEachN);
    ngx::Log("#\t iterations  {}", incIterations);

    SparseOptimizer::VertexIDMap vertices = optimizer.vertices();
    for (SparseOptimizer::VertexIDMap::const_iterator it = vertices.begin();
         it != vertices.end(); ++it) {
      const SparseOptimizer::Vertex* v =
          dynamic_cast<const SparseOptimizer::Vertex*>(it->second);
      maxDim = max(maxDim, v->dimension());
    }

    vector<SparseOptimizer::Edge*> edges;
    for (auto it : optimizer.edges()) {
      SparseOptimizer::Edge* e = dynamic_cast<SparseOptimizer::Edge*>(it);
      edges.push_back(e);
    }
    optimizer.edges().clear();
    optimizer.vertices().clear();
    optimizer.setVerbose(false);

    // sort the edges in a way that inserting them makes sense
    sort(edges.begin(), edges.end(), IncrementalEdgesCompare());

    double cumTime = 0.;
    int vertexCount = 0;
    int lastOptimizedVertexCount = 0;
    int lastVisUpdateVertexCount = 0;
    bool freshlyOptimized = false;
    bool firstRound = true;
    HyperGraph::VertexSet verticesAdded;
    HyperGraph::EdgeSet edgesAdded;
    for (auto e : edges) {
      int doInit = 0;
      SparseOptimizer::Vertex* v1 = optimizer.vertex(e->vertices()[0]->id());
      SparseOptimizer::Vertex* v2 = optimizer.vertex(e->vertices()[1]->id());

      if (v1 == nullptr) {
        SparseOptimizer::Vertex* v = v1 =
            dynamic_cast<SparseOptimizer::Vertex*>(e->vertices()[0]);
        bool v1Added = optimizer.addVertex(v);
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v1Added);
        if (!v1Added)
          ngx::Log(ngx::Error, "Error adding vertex {}", v->id());
        else
          verticesAdded.insert(v);
        doInit = 1;
        if (v->dimension() == maxDim) vertexCount++;
      }

      if (v2 == nullptr) {
        SparseOptimizer::Vertex* v = v2 =
            dynamic_cast<SparseOptimizer::Vertex*>(e->vertices()[1]);
        bool v2Added = optimizer.addVertex(v);
        // cerr << "adding" << v->id() << "(" << v->dimension() << ")" << endl;
        assert(v2Added);
        if (!v2Added)
          ngx::Log(ngx::Error, "Error adding vertex {}", v->id());
        else
          verticesAdded.insert(v);
        doInit = 2;
        if (v->dimension() == maxDim) vertexCount++;
      }

      // adding the edge and initialization of the vertices
      {
        // cerr << " adding edge " << e->vertices()[0]->id() <<  " " <<
        // e->vertices()[1]->id() << endl;
        if (!optimizer.addEdge(e)) {
          ngx::Log(ngx::Error, "Unable to add edge {} -> {}",
                   e->vertices()[0]->id(), e->vertices()[1]->id());
        } else {
          edgesAdded.insert(e);
        }

        if (doInit != 0) {
          OptimizableGraph::Vertex* from =
              dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
          OptimizableGraph::Vertex* to =
              dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
          switch (doInit) {
            case 1:  // initialize v1 from v2
            {
              HyperGraph::VertexSet toSet;
              toSet.insert(to);
              if (e->initialEstimatePossible(toSet, from) > 0.) {
                // cerr << "init: "
                //<< to->id() << "(" << to->dimension() << ") -> "
                //<< from->id() << "(" << from->dimension() << ") " << endl;
                e->initialEstimate(toSet, from);
              } else {
                assert(0 && "Added unitialized variable to the graph");
              }
              break;
            }
            case 2: {
              HyperGraph::VertexSet fromSet;
              fromSet.insert(from);
              if (e->initialEstimatePossible(fromSet, to) > 0.) {
                // cerr << "init: "
                //<< from->id() << "(" << from->dimension() << ") -> "
                //<< to->id() << "(" << to->dimension() << ") " << endl;
                e->initialEstimate(fromSet, to);
              } else {
                assert(0 && "Added unitialized variable to the graph");
              }
              break;
            }
            default:
              ngx::Log(ngx::Error, "doInit wrong value");
          }
        }
      }

      freshlyOptimized = false;
      {
        // cerr << "Optimize" << endl;
        if (vertexCount - lastOptimizedVertexCount >= updateGraphEachN) {
          if (firstRound) {
            if (!optimizer.initializeOptimization()) {
              ngx::Log(ngx::Error, "initialization failed");
              return 0;
            }
          } else {
            if (!optimizer.updateInitialization(verticesAdded, edgesAdded)) {
              ngx::Log(ngx::Error, "updating initialization failed");
              return 0;
            }
          }
          verticesAdded.clear();
          edgesAdded.clear();
          double ts = get_monotonic_time();
          int currentIt = optimizer.optimize(incIterations, !firstRound);
          double dts = get_monotonic_time() - ts;
          cumTime += dts;
          firstRound = false;
          // optimizer->setOptimizationTime(cumTime);
          if (verbose) {
            double chi2 = optimizer.chi2();
            ngx::Log(
                "nodes= {}\t edges= {}\t chi2= {}\t time= {}\t iterations= "
                "{}\t cumTime= {}",
                optimizer.vertices().size(), optimizer.edges().size(), chi2,
                dts, currentIt, cumTime);
          }
          lastOptimizedVertexCount = vertexCount;
          freshlyOptimized = true;

          if (guiOut) {
            if (vertexCount - lastVisUpdateVertexCount >= updateDisplayEveryN) {
              dumpEdges(cout, optimizer);
              lastVisUpdateVertexCount = vertexCount;
            }
          }
        }
      }

    }  // for all edges

    if (!freshlyOptimized) {
      double ts = get_monotonic_time();
      int currentIt = optimizer.optimize(incIterations, !firstRound);
      double dts = get_monotonic_time() - ts;
      cumTime += dts;
      // optimizer->setOptimizationTime(cumTime);
      if (verbose) {
        double chi2 = optimizer.chi2();
        ngx::Log(
            "nodes= {}\t edges= {}\t chi2= {}\t time= {}\t iterations= {}\t "
            "cumTime= {}",
            optimizer.vertices().size(), optimizer.edges().size(), chi2, dts,
            currentIt, cumTime);
      }
    }

  } else {
    // BATCH optimization

    if (statsFile != "") {
      // allocate buffer for statistics;
      optimizer.setComputeBatchStatistics(true);
    }
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    double loadChi = optimizer.chi2();
    ngx::Log("Initial chi2 = {:f}", loadChi);

    if (initialGuess) {
      optimizer.computeInitialGuess();
    } else if (initialGuessOdometry) {
      EstimatePropagatorCostOdometry costFunction(&optimizer);
      optimizer.computeInitialGuess(costFunction);
    }
    double initChi = optimizer.chi2();

    signal(SIGINT, sigquit_handler);
    int result = optimizer.optimize(maxIterations);
    if (maxIterations > 0 && result == OptimizationAlgorithm::Fail) {
      ngx::Log(ngx::Error, "Cholesky failed, result might be invalid");
    } else if (computeMarginals) {
      std::vector<std::pair<int, int> > blockIndices;
      for (auto v : optimizer.activeVertices()) {
        if (v->hessianIndex() >= 0) {
          blockIndices.push_back(
              make_pair(v->hessianIndex(), v->hessianIndex()));
        }
        if (v->hessianIndex() > 0) {
          blockIndices.push_back(
              make_pair(v->hessianIndex() - 1, v->hessianIndex()));
        }
      }
      SparseBlockMatrix<MatrixXd> spinv;
      if (optimizer.computeMarginals(spinv, blockIndices)) {
        for (auto v : optimizer.activeVertices()) {
          ngx::Log("Vertex id:{}", v->id());
          if (v->hessianIndex() >= 0) {
            ngx::Log(
                "inv block :{}, {}\n{}", v->hessianIndex(), v->hessianIndex(),
                Streamed{*spinv.block(v->hessianIndex(), v->hessianIndex())});
          }
          if (v->hessianIndex() > 0) {
            ngx::Log("inv block :{}, {}\n{}", v->hessianIndex() - 1,
                     v->hessianIndex(),
                     Streamed{*spinv.block(v->hessianIndex() - 1,
                                           v->hessianIndex())});
          }
        }
      }
    }

    optimizer.computeActiveErrors();
    double finalChi = optimizer.chi2();

    if (summaryFile != "") {
      PropertyMap summary;
      summary.makeProperty<StringProperty>("filename", inputFilename);
      summary.makeProperty<IntProperty>("n_vertices",
                                        optimizer.vertices().size());
      summary.makeProperty<IntProperty>("n_edges", optimizer.edges().size());

      int nLandmarks = 0;
      int nPoses = 0;
      int maxDim = *vertexDimensions.rbegin();
      for (auto& it : optimizer.vertices()) {
        OptimizableGraph::Vertex* v =
            dynamic_cast<OptimizableGraph::Vertex*>(it.second);
        if (v->dimension() != maxDim) {
          nLandmarks++;
        } else
          nPoses++;
      }
      set<string> edgeTypes;
      for (auto it : optimizer.edges()) {
        edgeTypes.insert(Factory::instance()->tag(it));
      }
      stringstream edgeTypesString;
      for (const auto& edgeType : edgeTypes) {
        edgeTypesString << edgeType << " ";
      }

      summary.makeProperty<IntProperty>("n_poses", nPoses);
      summary.makeProperty<IntProperty>("n_landmarks", nLandmarks);
      summary.makeProperty<StringProperty>("edge_types", edgeTypesString.str());
      summary.makeProperty<DoubleProperty>("load_chi", loadChi);
      summary.makeProperty<StringProperty>("solver", strSolver);
      summary.makeProperty<BoolProperty>("robustKernel", !robustKernel.empty());
      summary.makeProperty<DoubleProperty>("init_chi", initChi);
      summary.makeProperty<DoubleProperty>("final_chi", finalChi);
      summary.makeProperty<IntProperty>("maxIterations", maxIterations);
      summary.makeProperty<IntProperty>("realIterations", result);
      ofstream os;
      os.open(summaryFile.c_str(), ios::app);
      summary.writeToCSV(os);
    }

    if (statsFile != "") {
      ngx::Log("writing stats to file \"{}\"", statsFile);
      ofstream os(statsFile.c_str());
      const BatchStatisticsContainer& bsc = optimizer.batchStatistics();

      for (int i = 0; i < maxIterations; i++) {
        os << bsc[i] << endl;
      }
      ngx::Log("done.");
    }
  }

  // saving again
  if (!gnudump.empty()) {
    bool gnuPlotStatus = saveGnuplot(gnudump, optimizer);
    if (!gnuPlotStatus) {
      ngx::Log(ngx::Error, "Error while writing gnuplot files");
    }
  }

  if (!outputfilename.empty()) {
    if (outputfilename == "-") {
      ngx::Log("saving to stdout");
      optimizer.save(cout);
    } else {
      ngx::Log("saving {}", outputfilename);
      optimizer.save(outputfilename.c_str());
    }
    ngx::Log("done.");
  }

  // destroy all the singletons
  // Factory::destroy();
  // OptimizationAlgorithmFactory::destroy();
  // HyperGraphActionLibrary::destroy();

  return 0;
}
