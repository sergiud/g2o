// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "properties_widget.h"

#include <g2o/stuff/logger.h>
#include <g2o/stuff/property.h>

#include <QLineEdit>
#include <cassert>

using namespace std;

using namespace g2o;

PropertiesWidget::PropertiesWidget(QWidget* parent, Qt::WindowFlags f)
    : QDialog(parent, f), _properties(nullptr) {
  setupUi(this);
}

PropertiesWidget::~PropertiesWidget() = default;

void PropertiesWidget::updateDisplayedProperties() {
  tableWidget->clear();
  _propNames.clear();

  tableWidget->setColumnCount(2);

  QStringList horizontalHeaders;
  horizontalHeaders.append("Name");
  horizontalHeaders.append("Value");
  tableWidget->setHorizontalHeaderLabels(horizontalHeaders);

  tableWidget->verticalHeader()->hide();

  PropertyMap* properties = _properties;
  if (properties == nullptr) return;
  tableWidget->setRowCount(properties->size());

  int r = 0;
  for (auto it = properties->begin(); it != properties->end(); ++it, ++r) {
    auto* textItem = new QTableWidgetItem;
    textItem->setText(QString::fromStdString(humanReadablePropName(it->first)));
    textItem->setFlags(textItem->flags() & ~Qt::ItemIsEditable);
    tableWidget->setItem(r, 0, textItem);
    _propNames.push_back(it->first);

    if (dynamic_cast<Property<bool>*>(it->second) != nullptr) {
      Property<bool>* prop = dynamic_cast<Property<bool>*>(it->second);
      auto* checkItem = new QTableWidgetItem;
      checkItem->setText("enabled");
      checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable);
      if (prop->value())
        checkItem->setCheckState(Qt::Checked);
      else
        checkItem->setCheckState(Qt::Unchecked);
      tableWidget->setItem(r, 1, checkItem);
    } else {
      auto* editor = new QLineEdit(tableWidget);
      editor->setText(QString::fromStdString(it->second->toString()));
      if (dynamic_cast<Property<int>*>(it->second) != nullptr) {
        editor->setValidator(new QIntValidator(editor));
      } else if (dynamic_cast<Property<float>*>(it->second) != nullptr) {
        editor->setValidator(new QDoubleValidator(editor));
      } else if (dynamic_cast<Property<double>*>(it->second) != nullptr) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      tableWidget->setCellWidget(r, 1, editor);
    }
  }
  tableWidget->resizeColumnToContents(0);
}

void PropertiesWidget::applyProperties() {
  assert(tableWidget->rowCount() == (int)_propNames.size());
  PropertyMap* properties = _properties;
  for (int r = 0; r < tableWidget->rowCount(); ++r) {
    const std::string& propName = _propNames[r];
    BaseProperty* baseProp = properties->getProperty<BaseProperty>(propName);
    if (baseProp == nullptr) continue;

    if (dynamic_cast<Property<bool>*>(baseProp) != nullptr) {
      Property<bool>* prop = dynamic_cast<Property<bool>*>(baseProp);
      QTableWidgetItem* checkItem = tableWidget->item(r, 1);
      prop->setValue(checkItem->checkState() == Qt::Checked);
    } else {
      QLineEdit* editor =
          dynamic_cast<QLineEdit*>(tableWidget->cellWidget(r, 1));
      bool status = baseProp->fromString(editor->text().toStdString());
      if (!status) {
        ngx::Log(ngx::Warning, "unable to set property {}", baseProp->name());
      }
    }
  }
}

void PropertiesWidget::on_btnApply_clicked() { applyProperties(); }

void PropertiesWidget::on_btnOK_clicked() {
  applyProperties();
  close();
}

std::string PropertiesWidget::humanReadablePropName(
    const std::string& propertyName) const {
  return propertyName;
}

void PropertiesWidget::setProperties(PropertyMap* properties) {
  _properties = properties;
  updateDisplayedProperties();
}
