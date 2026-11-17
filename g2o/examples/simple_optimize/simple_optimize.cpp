// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <iostream>
// #include <g2o/types/slam3d/types_slam3d.h>
// #include <g2o/types/slam2d/types_slam2d.h>

#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>

using namespace std;
using namespace g2o;

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  // Command line parsing
  int maxIterations;
  string outputFilename;
  string inputFilename;
  CommandArgs arg;
  arg.param("i", maxIterations, 10,
            "perform n iterations, if negative consider the gain");
  arg.param("o", outputFilename, "", "output final version of the graph");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed");
  arg.parseArgs(argc, argv);

  // create the linear solver
  BlockSolverX::LinearSolverType* linearSolver =
      new LinearSolverCSparse<BlockSolverX::PoseMatrixType>();

  // create the block solver on top of the linear solver
  auto* blockSolver = new BlockSolverX(linearSolver);

  // create the algorithm to carry out the optimization
  // OptimizationAlgorithmGaussNewton* optimizationAlgorithm = new
  // OptimizationAlgorithmGaussNewton(blockSolver);
  auto* optimizationAlgorithm = new OptimizationAlgorithmLevenberg(blockSolver);

  // NOTE: We skip to fix a variable here, either this is stored in the file
  // itself or Levenberg will handle it.

  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  optimizer.setVerbose(true);
  optimizer.setAlgorithm(optimizationAlgorithm);

  ifstream ifs(inputFilename.c_str());
  if (!ifs) {
    ngx::Log(ngx::Error, "unable to open {}", inputFilename);
    return 1;
  }
  optimizer.load(ifs);
  optimizer.initializeOptimization();
  optimizer.optimize(maxIterations);

  if (!outputFilename.empty()) {
    if (outputFilename == "-") {
      ngx::Log("saving to stdout");
      optimizer.save(cout);
    } else {
      ngx::Log("saving {}", outputFilename);
      optimizer.save(outputFilename.c_str());
    }
    ngx::Log("done.");
  }
  return 0;
}
