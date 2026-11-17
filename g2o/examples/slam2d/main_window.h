// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_MAIN_WINDOW_H
#define G2O_MAIN_WINDOW_H

#include "ui_base_main_window.h"

G2O_START_NAMESPACE
class OptimizationAlgorithm;
G2O_END_NAMESPACE

class MainWindow : public QMainWindow, public Ui::BaseMainWindow {
  Q_OBJECT
 public:
  MainWindow(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~MainWindow();

  g2o::OptimizationAlgorithm* solverGaussNewton{};
  g2o::OptimizationAlgorithm* solverLevenberg{};

 public slots:
  void on_actionLoad_triggered(bool);
  void on_actionSave_triggered(bool);
  void on_actionQuit_triggered(bool);
  void on_btnOptimize_clicked();
  void on_btnInitialGuess_clicked();

 protected:
  void fixGraph();
};

#endif
