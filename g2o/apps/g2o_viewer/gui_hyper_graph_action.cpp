// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "gui_hyper_graph_action.h"

#include "g2o_qglviewer.h"

#include <QApplication>

G2O_START_NAMESPACE

GuiHyperGraphAction::GuiHyperGraphAction() :
  HyperGraphAction(),
  viewer(nullptr), dumpScreenshots(false)
{
}

GuiHyperGraphAction::~GuiHyperGraphAction()
= default;

HyperGraphAction* GuiHyperGraphAction::operator()(const HyperGraph* graph, Parameters* parameters)
{
  (void) graph;
  if (viewer != nullptr) {
    viewer->setUpdateDisplay(true);
    viewer->updateDisplay();

    if (dumpScreenshots) {
      ParametersIteration* p = dynamic_cast<ParametersIteration*>(parameters);
      if (p != nullptr) {
        viewer->setSnapshotFormat(QString("PNG"));
        viewer->setSnapshotQuality(-1);
        viewer->saveSnapshot(QString::asprintf("g2o%.6d.png", p->iteration), true);
      }
    }

    qApp->processEvents();
    return this;
  }
  return nullptr;
}

G2O_END_NAMESPACE
