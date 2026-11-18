// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <g2o/core/factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/logger.h>

#include <QApplication>
#include <memory>

#include "main_window.h"
using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  QApplication qapp(argc, argv);

  // The graph outlives the window that shows it.
  auto graph = std::make_unique<SparseOptimizer>();
  MainWindow mw;
  mw.show();

  mw.viewer->graph = graph.get();

  mw.viewer->graph->setAlgorithm(mw.createAlgorithm());

  return qapp.exec();
}
