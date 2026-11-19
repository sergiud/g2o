// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "main_window.h"

#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>

#include <QApplication>
#include <QComboBox>
#include <QString>
#include <QTableWidget>
#include <string>
#include <vector>

#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "properties_widget.h"

G2O_USE_OPTIMIZATION_LIBRARY(eigen)
G2O_USE_TYPE_GROUP(slam2d)

using ::testing::Contains;
using ::testing::NotNull;

namespace {

//! Selects the solver with the given name in the list of the window.
void selectSolver(MainWindow& window, const QString& name) {
  const int index = window.coOptimizer->findText(name);
  ASSERT_GE(index, 0) << name.toStdString() << " is not available";
  window.coOptimizer->setCurrentIndex(index);
}

//! Estimates of the planar poses of the graph in the order of their ids.
std::vector<Eigen::Vector3d> estimates(const g2o::SparseOptimizer& optimizer) {
  std::vector<Eigen::Vector3d> poses;
  for (int id = 0; id < static_cast<int>(optimizer.vertices().size()); ++id) {
    poses.push_back(static_cast<const g2o::VertexSE2*>(optimizer.vertex(id))
                        ->estimate()
                        .toVector());
  }
  return poses;
}

}  // namespace

/**
 * The dialog of the solver properties stays open while the user optimizes
 * with another solver, which replaces the solver the dialog shows.
 */
TEST(Viewer, PropertiesDialogFollowsTheSolver) {
  int argc = 1;
  char name[] = "unittest_viewer";
  char* argv[] = {name, nullptr};
  QApplication application(argc, argv);
  g2o::SparseOptimizer optimizer;
  MainWindow window;
  window.updateDisplayedSolvers();
  window.viewer->graph = &optimizer;
  ASSERT_TRUE(window.loadFromFile(QString::fromUtf8(G2O_TEST_GRAPH)));
  selectSolver(window, "gn_var_eigen");
  window.on_btnOptimizerParamaters_clicked();

  selectSolver(window, "lm_var_eigen");
  window.on_btnOptimize_clicked();

  const auto* dialog = window.findChild<PropertiesWidget*>();
  ASSERT_THAT(dialog, NotNull());
  std::vector<std::string> names;
  for (int row = 0; row < dialog->tableWidget->rowCount(); ++row) {
    names.push_back(dialog->tableWidget->item(row, 0)->text().toStdString());
  }
  EXPECT_THAT(names, Contains("initialLambda"));
}

/**
 * Reloading the graph replaces its elements, which the solver has to prepare
 * again before it optimizes them.
 */
TEST(Viewer, OptimizesAReloadedGraph) {
  int argc = 1;
  char name[] = "unittest_viewer";
  char* argv[] = {name, nullptr};
  QApplication application(argc, argv);
  g2o::SparseOptimizer optimizer;
  MainWindow window;
  window.updateDisplayedSolvers();
  window.viewer->graph = &optimizer;
  ASSERT_TRUE(window.loadFromFile(QString::fromUtf8(G2O_TEST_GRAPH)));
  selectSolver(window, "lm_var_eigen");
  window.on_btnOptimize_clicked();
  const std::vector<Eigen::Vector3d> expected = estimates(optimizer);

  window.on_btnReload_clicked();
  window.on_btnOptimize_clicked();

  EXPECT_EQ(estimates(optimizer), expected);
}
