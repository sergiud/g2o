// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <g2o/apps/g2o_cli/dl_wrapper.h>
#include <g2o/apps/g2o_cli/g2o_common.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>

#include <QApplication>

#include "run_g2o_viewer.h"
using namespace std;
using namespace g2o;

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  OptimizableGraph::initMultiThreading();
  QApplication qapp(argc, argv);

  CommandArgs arg;
#ifndef G2O_DISABLE_DYNAMIC_LOADING_OF_LIBRARIES
  string dummy;
  arg.param("solverlib", dummy, "",
            "specify a solver library which will be loaded");
  arg.param("typeslib", dummy, "",
            "specify a types library which will be loaded");
  // loading the standard solver /  types
  DlWrapper dlTypesWrapper;
  loadStandardTypes(dlTypesWrapper, argc, argv);
  // register all the solvers
  DlWrapper dlSolverWrapper;
  loadStandardSolver(dlSolverWrapper, argc, argv);
#endif

  // run the viewer
  return RunG2OViewer::run(argc, argv, arg);
}
