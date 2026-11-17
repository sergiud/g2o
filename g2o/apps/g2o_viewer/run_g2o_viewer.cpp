// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "run_g2o_viewer.h"

#include <g2o/apps/g2o_cli/dl_wrapper.h>
#include <g2o/config.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/opengl_wrapper.h>

#include <QApplication>
#include <QThread>

#include "gui_hyper_graph_action.h"
#include "main_window.h"
#include "stream_redirect.h"

G2O_START_NAMESPACE

/**
 * \brief helper for calling usleep on any system using Qt
 */
class SleepThread : public QThread {
 public:  // make the proctected methods publicly available
  using QThread::msleep;
  using QThread::usleep;
};

int RunG2OViewer::run(int argc, char** argv, CommandArgs& arg) {
  std::string inputFilename;
  std::string loadLookup;
  arg.param("renameTypes", loadLookup, "",
            "create a lookup for loading types into other types,\n\t "
            "TAG_IN_FILE=INTERNAL_TAG_FOR_TYPE,TAG2=INTERNAL2\n\t e.g., "
            "VERTEX_CAM=VERTEX_SE3:EXPMAP");
  arg.paramLeftOver("graph-input", inputFilename, "",
                    "graph file which will be processed", true);
  arg.parseArgs(argc, argv);

  MainWindow mw;
  mw.updateDisplayedSolvers();
  mw.updateRobustKernels();
  mw.show();

  // redirect the output that normally goes to cerr to the textedit in the
  // viewer
  StreamRedirect redirect(std::cerr, mw.plainTextEdit);

  // setting up the optimizer
  auto* optimizer = new SparseOptimizer();
  // Loading the input data
  if (loadLookup.size() > 0) {
    optimizer->setRenamedTypesFromString(loadLookup);
  }
  mw.viewer->graph = optimizer;

  // set up the GUI action
  GuiHyperGraphAction guiHyperGraphAction;
  guiHyperGraphAction.viewer = mw.viewer;
  // optimizer->addPostIterationAction(&guiHyperGraphAction);
  optimizer->addPreIterationAction(&guiHyperGraphAction);

  if (!inputFilename.empty()) {
    mw.loadFromFile(QString::fromStdString(inputFilename));
  }

  QCoreApplication* myapp = QApplication::instance();
  while (mw.isVisible()) {
    guiHyperGraphAction.dumpScreenshots = mw.actionDump_Images->isChecked();
    if (myapp != nullptr) myapp->processEvents();
    SleepThread::msleep(10);
  }

  delete optimizer;
  return 0;
}

G2O_END_NAMESPACE
