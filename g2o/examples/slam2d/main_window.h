// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_MAIN_WINDOW_H
#define G2O_MAIN_WINDOW_H

#include <g2o/core/optimization_algorithm.h>

#include <memory>

#include "ui_base_main_window.h"

class MainWindow : public QMainWindow, public Ui::BaseMainWindow {
  Q_OBJECT
 public:
  MainWindow(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~MainWindow();

  //! creates the optimization algorithm that the window selects
  std::unique_ptr<g2o::OptimizationAlgorithm> createAlgorithm() const;

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
