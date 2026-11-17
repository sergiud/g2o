// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_PROPERTIES_WINDOW_H
#define G2O_PROPERTIES_WINDOW_H

#include <string>
#include <vector>

#include <QDialog>

#include <g2o/core/abi.h>
#include <g2o/viewer/g2o_viewer_api.h>

#include "ui_base_properties_widget.h"

#include <g2o/prefix.h>

G2O_START_NAMESPACE
  class G2oQGLViewer;
  class PropertyMap;
G2O_END_NAMESPACE

#include <g2o/suffix.h>


class G2O_VIEWER_API PropertiesWidget : public QDialog, public Ui::BasePropertiesWidget
{
  Q_OBJECT
  public:
    PropertiesWidget(QWidget * parent = nullptr, Qt::WindowFlags f = {});
    virtual ~PropertiesWidget();

    void setProperties(g2o::PropertyMap* properties);

  public slots:
    void on_btnApply_clicked();
    void on_btnOK_clicked();

  protected:
    std::vector<std::string> _propNames;
    g2o::PropertyMap* _properties;

    virtual void updateDisplayedProperties();
    virtual void applyProperties();
    virtual std::string humanReadablePropName(const std::string& propertyName) const;
};

#endif
