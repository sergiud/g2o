// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2020 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "viewer_properties_widget.h"

#include "g2o_qglviewer.h"
#include <g2o/stuff/property.h>

#include <QLineEdit>

#include <cassert>
#include <iostream>

#ifdef __GNUC__
  #include <cxxabi.h>
#endif

using namespace std;

using namespace g2o;

/**
 * demangle the name of a type, depends on the used compiler
 */
static std::string demangleName(const std::string& fullPropName)
{
#ifdef __GNUC__
  // find :: and extract the mangled class name from the whole string
  string mangledName;
  string propName;
  string::size_type found = fullPropName.rfind("::");
  if (found != string::npos) {
    mangledName = fullPropName.substr(0, found);
    propName    = fullPropName.substr(found);
  } else {
    mangledName = propName;
  }

  int status;
  char* s = abi::__cxa_demangle(mangledName.c_str(), nullptr, nullptr, &status);
  if (status != 0) {
    free(s);
    return fullPropName;
  } else {
    std::string demangled(s);
    free(s);
    return demangled + propName;
  }
#else
  // TODO for other compilers
  return fullPropName;
#endif
}

ViewerPropertiesWidget::ViewerPropertiesWidget(QWidget * parent, Qt::WindowFlags f) :
  PropertiesWidget(parent, f)
{
}

ViewerPropertiesWidget::~ViewerPropertiesWidget()
= default;

void ViewerPropertiesWidget::applyProperties()
{
  PropertiesWidget::applyProperties();

  // draw with the new properties
  _viewer->setUpdateDisplay(true);
  _viewer->updateDisplay();
}

void ViewerPropertiesWidget::setViewer(g2o::G2oQGLViewer* viewer)
{
  _viewer = viewer;
  setProperties(viewer->parameters());
}

std::string ViewerPropertiesWidget::humanReadablePropName(const std::string& propertyName) const
{
  return demangleName(propertyName);
}
