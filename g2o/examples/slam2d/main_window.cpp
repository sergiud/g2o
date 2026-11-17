// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2020 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "main_window.h"
// #include "moc_main_window.cpp"

#include <g2o/core/estimate_propagator.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/logger.h>

#include <QFileDialog>
#include <fstream>
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
    g2o::ngx::Log("Graph loaded with {} vertices and {} measurments",
                  viewer->graph->vertices().size(),
                  viewer->graph->edges().size());
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
      g2o::ngx::Log("Saved {}", filename.toStdString());
    else
      g2o::ngx::Log(g2o::ngx::Error, "Error while saving file");
  }
}

void MainWindow::on_actionQuit_triggered(bool) { close(); }

void MainWindow::on_btnOptimize_clicked() {
  if (viewer->graph->vertices().empty() || viewer->graph->edges().empty()) {
    g2o::ngx::Log(g2o::ngx::Error, "Graph has no vertices / egdes");
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
    g2o::ngx::Log(g2o::ngx::Error,
                  "Optimization failed, result might be invalid");
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
      g2o::ngx::Log(g2o::ngx::Error,
                    "cannot find a vertex to fix in this thing");
      return;
    } else {
      g2o::ngx::Log("graph is fixed by node {}", gauge->id());
      gauge->setFixed(true);
    }
  } else {
    g2o::ngx::Log("graph is fixed by priors");
  }

  viewer->graph->setVerbose(true);
  viewer->graph->computeActiveErrors();
}
