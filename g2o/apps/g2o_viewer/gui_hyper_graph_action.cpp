// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

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
        viewer->saveSnapshot(QString().sprintf("g2o%.6d.png", p->iteration), true);
      }
    }

    qApp->processEvents();
    return this;
  }
  return nullptr;
}

G2O_END_NAMESPACE
