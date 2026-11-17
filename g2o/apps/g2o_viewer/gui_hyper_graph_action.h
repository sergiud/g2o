// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_GUI_HYPER_GRAPH_ACTION_H
#define G2O_GUI_HYPER_GRAPH_ACTION_H

#include <g2o/core/abi.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/viewer/g2o_viewer_api.h>

G2O_START_NAMESPACE

class G2oQGLViewer;

/**
 * \brief action which calls an GUI update after each iteration
 */
class G2O_VIEWER_API GuiHyperGraphAction : public HyperGraphAction {
 public:
  GuiHyperGraphAction();
  ~GuiHyperGraphAction();

  /**
   * calling updateGL, processEvents to visualize the current state after each
   * iteration
   */
  HyperGraphAction* operator()(const HyperGraph* graph,
                               Parameters* parameters = nullptr);

  G2oQGLViewer* viewer;  ///< the viewer which visualizes the graph
  bool dumpScreenshots;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
