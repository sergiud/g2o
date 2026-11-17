// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_QGL_GRAPH_VIEWER_H
#define G2O_QGL_GRAPH_VIEWER_H

#include <g2o/core/abi.h>

#include "qglviewer.h"

G2O_START_NAMESPACE

class SparseOptimizer;

class Slam2DViewer : public QGLViewer {
 public:
  explicit Slam2DViewer(QWidget* parent = nullptr, Qt::WindowFlags flags = {});
  ~Slam2DViewer();
  virtual void draw();
  void init();

 public:
  SparseOptimizer* graph;
  bool drawCovariance;
};

G2O_END_NAMESPACE

#endif
