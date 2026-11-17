// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_VIEWER_PROPERTIES_WIDGET_H
#define G2O_VIEWER_PROPERTIES_WIDGET_H

#include <g2o/prefix.h>
#include <g2o/viewer/g2o_viewer_api.h>

#include "properties_widget.h"

G2O_START_NAMESPACE
class G2oQGLViewer;
G2O_END_NAMESPACE

#include <g2o/suffix.h>

class G2O_VIEWER_API ViewerPropertiesWidget : public PropertiesWidget {
 public:
  ViewerPropertiesWidget(QWidget* parent = nullptr, Qt::WindowFlags f = {});
  virtual ~ViewerPropertiesWidget();

  void setViewer(g2o::G2oQGLViewer* viewer);

 protected:
  g2o::G2oQGLViewer* _viewer{};

  virtual void applyProperties();
  virtual std::string humanReadablePropName(const std::string& probName) const;
};

#endif
