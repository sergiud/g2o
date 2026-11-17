// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <iostream>
#include <iomanip>
#include <csignal>
#include <cstdlib>

#include <g2o/stuff/macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/timeutil.h>

#include "slam_parser/interface/parser_interface.h"

#include <g2o_interactive/g2o_slam_interface.h>
#include "graph_optimizer_sparse_online.h"

static bool hasToStop=false;

using namespace std;
using namespace g2o;

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

int main(int argc, char** argv)
{
  bool pcg;
  int updateEachN;
  bool vis;
  bool verbose;
  // command line parsing
  CommandArgs arg;
  arg.param("update", updateEachN, 10, "update the graph after inserting N nodes");
  arg.param("pcg", pcg, false, "use PCG instead of Cholesky");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("g", vis, false, "gnuplot visualization");

  arg.parseArgs(argc, argv);

  SparseOptimizerOnline optimizer(pcg);
  //SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);
  optimizer.setForceStopFlag(&hasToStop);
  optimizer.vizWithGnuplot = vis;

  G2oSlamInterface slamInterface(&optimizer);
  slamInterface.setUpdateGraphEachN(updateEachN);

  SlamParser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(cin))
  {
    // do something additional if needed
  }

  return 0;
}
