// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_MAIN_WINDOW_H
#define G2O_MAIN_WINDOW_H

#include <g2o/core/abi.h>
#include <g2o/core/optimization_algorithm_property.h>
#include <g2o/viewer/g2o_viewer_api.h>

#include <vector>

#include "ui_base_main_window.h"

class ViewerPropertiesWidget;
class PropertiesWidget;

#include <g2o/prefix.h>

G2O_START_NAMESPACE
class DlWrapper;
class OptimizationAlgorithm;
G2O_END_NAMESPACE

#include <g2o/suffix.h>

/**
 * \brief main window of the g2o viewer
 */
class G2O_VIEWER_API MainWindow : public QMainWindow,
                                  public Ui::BaseMainWindow {
  Q_OBJECT
 public:
  MainWindow(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~MainWindow();

 public:
  /**
   * list the available solvers in the GUI
   */
  void updateDisplayedSolvers();

  /**
   * list the available robust kernels in the GUI
   */
  void updateRobustKernels();

  /**
   * load a graph on which we will operate from a file
   */
  bool loadFromFile(const QString& filename);

 public slots:
  void on_actionLoad_triggered(bool);
  void on_actionSave_triggered(bool);
  void on_actionQuit_triggered(bool);
  void on_actionWhite_Background_triggered(bool);
  void on_actionDefault_Background_triggered(bool);
  void on_actionProperties_triggered(bool);
  void on_actionSave_Screenshot_triggered(bool);
  void on_actionLoad_Viewer_State_triggered(bool);
  void on_actionSave_Viewer_State_triggered(bool);

  void on_btnOptimize_clicked();
  void on_btnInitialGuess_clicked();
  void on_btnSetZero_clicked();
  void on_btnForceStop_clicked();
  void on_btnOptimizerParamaters_clicked();
  void on_btnReload_clicked();

 protected:
  void fixGraph();
  bool allocateSolver(bool& allocatedNewSolver);
  bool prepare();
  void setRobustKernel();
  bool load(const QString& filename);

  std::vector<g2o::OptimizationAlgorithmProperty> _knownSolvers;
  int _lastSolver;
  bool _forceStopFlag{};
  g2o::OptimizationAlgorithmProperty _currentOptimizationAlgorithmProperty;
  g2o::OptimizationAlgorithm* _currentSolver;

  ViewerPropertiesWidget* _viewerPropertiesWidget;
  PropertiesWidget* _optimizerPropertiesWidget;
  std::string _filename;
};

#endif
