// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <QApplication>
#include <iostream>

#include "main_window.h"
using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);

int main(int argc, char** argv) {
  QApplication qapp(argc, argv);

  MainWindow mw;
  mw.show();

  mw.viewer->graph = new SparseOptimizer();

  typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  auto* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  auto* blockSolver = new SlamBlockSolver(linearSolver);

  auto* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
  auto* solverLevenberg = new OptimizationAlgorithmLevenberg(blockSolver);
  mw.solverGaussNewton = solverGauss;
  mw.solverLevenberg = solverLevenberg;
  mw.viewer->graph->setAlgorithm(solverGauss);

  return qapp.exec();
}
