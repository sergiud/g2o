// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_QGL_GRAPH_VIEWER_H
#define G2O_QGL_GRAPH_VIEWER_H

#include <g2o/core/abi.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/viewer/g2o_viewer_api.h>

#include <memory>

#include "qglviewer.h"

G2O_START_NAMESPACE

class SparseOptimizer;

/**
 * \brief OpenGL based viewer for the graph
 */
class G2O_VIEWER_API G2oQGLViewer : public QGLViewer {
 public:
  explicit G2oQGLViewer(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~G2oQGLViewer();
  void draw();
  void init();

  /**
   * the viewer uses a display list to cache the drawing, use setUpdateDisplay()
   * to force the creation of an updated display list.
   */
  bool updateDisplay() const { return _updateDisplay; }
  void setUpdateDisplay(bool updateDisplay);

  DrawAction::Parameters* parameters() { return _drawActionParameters.get(); }

 public:
  SparseOptimizer* graph;

 protected:
  HyperGraphElementAction* _drawActions;
  GLuint _drawList;
  bool _updateDisplay{};
  std::unique_ptr<DrawAction::Parameters> _drawActionParameters;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
