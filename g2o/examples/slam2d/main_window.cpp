// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2020 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "main_window.h"
// #include "moc_main_window.cpp"

#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer.h>

#include <QFileDialog>
#include <fstream>
#include <iostream>
using namespace std;

MainWindow::MainWindow(QWidget* parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags) {
  setupUi(this);
}

MainWindow::~MainWindow() = default;

void MainWindow::on_actionLoad_triggered(bool) {
  viewer->graph->clear();
  QString filename = QFileDialog::getOpenFileName(
      this, "Load g2o file", "", "g2o files (*.g2o);;All Files (*)");
  if (!filename.isNull()) {
    ifstream ifs(filename.toStdString().c_str());
    viewer->graph->load(ifs);
    cerr << "Graph loaded with " << viewer->graph->vertices().size()
         << " vertices and " << viewer->graph->edges().size() << " measurments"
         << endl;
  }
  viewer->update();
  fixGraph();
}

void MainWindow::on_actionSave_triggered(bool) {
  QString filename = QFileDialog::getSaveFileName(this, "Save g2o file", "",
                                                  "g2o files (*.g2o)");
  if (!filename.isNull()) {
    ofstream fout(filename.toStdString().c_str());
    viewer->graph->save(fout);
    if (fout.good())
      cerr << "Saved " << filename.toStdString() << endl;
    else
      cerr << "Error while saving file" << endl;
  }
}

void MainWindow::on_actionQuit_triggered(bool) { close(); }

void MainWindow::on_btnOptimize_clicked() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    cerr << "Graph has no vertices / egdes" << endl;
    return;
  }

  viewer->graph->initializeOptimization();

  if (rbGauss->isChecked())
    viewer->graph->setAlgorithm(solverGaussNewton);
  else if (rbLevenberg->isChecked())
    viewer->graph->setAlgorithm(solverLevenberg);
  else
    viewer->graph->setAlgorithm(solverGaussNewton);

  int maxIterations = spIterations->value();
  int iter = viewer->graph->optimize(maxIterations);
  if (maxIterations > 0 && (iter == 0)) {
    cerr << "Optimization failed, result might be invalid" << endl;
  }

  if (cbCovariances->isChecked()) {
    // TODO
    // viewer->graph->solver()->computeMarginals();
  }
  viewer->drawCovariance = cbCovariances->isChecked();

  viewer->update();
}

void MainWindow::on_btnInitialGuess_clicked() {
  viewer->graph->computeInitialGuess();
  viewer->drawCovariance = false;
  viewer->update();
}

void MainWindow::fixGraph() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    return;
  }

  // check for vertices to fix to remove DoF
  bool gaugeFreedom = viewer->graph->gaugeFreedom();
  g2o::OptimizableGraph::Vertex* gauge = viewer->graph->findGauge();
  if (gaugeFreedom) {
    if (gauge == nullptr) {
      cerr << "cannot find a vertex to fix in this thing" << endl;
      return;
    } else {
      cerr << "graph is fixed by node " << gauge->id() << endl;
      gauge->setFixed(true);
    }
  } else {
    cerr << "graph is fixed by priors" << endl;
  }

  viewer->graph->setVerbose(true);
  viewer->graph->computeActiveErrors();
}
