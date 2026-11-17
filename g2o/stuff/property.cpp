// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/macros.h>
#include <g2o/stuff/property.h>
#include <g2o/stuff/string_tools.h>

#include <iostream>
#include <vector>
using namespace std;

G2O_START_NAMESPACE

BaseProperty::BaseProperty(const std::string name_) : _name(name_) {}

BaseProperty::~BaseProperty() = default;

bool PropertyMap::addProperty(BaseProperty* p) {
  std::pair<PropertyMapIterator, bool> result = insert(make_pair(p->name(), p));
  return result.second;
}

bool PropertyMap::eraseProperty(const std::string& name) {
  auto it = find(name);
  if (it == end()) return false;
  delete it->second;
  erase(it);
  return true;
}

PropertyMap::PropertyMap() = default;

PropertyMap::~PropertyMap() {
  for (auto& it : *this) {
    if (it.second != nullptr) delete it.second;
  }
}

bool PropertyMap::updatePropertyFromString(const std::string& name,
                                           const std::string& value) {
  auto it = find(name);
  if (it == end()) return false;
  it->second->fromString(value);
  return true;
}

void PropertyMap::writeToCSV(std::ostream& os) const {
  for (const auto& it : *this) {
    BaseProperty* p = it.second;
    os << p->name() << ", ";
  }
  os << std::endl;
  for (const auto& it : *this) {
    BaseProperty* p = it.second;
    os << p->toString() << ", ";
  }
  os << std::endl;
}

bool PropertyMap::updateMapFromString(const std::string& values) {
  bool status = true;
  vector<string> valuesMap = strSplit(values, ",");
  for (auto& i : valuesMap) {
    vector<string> m = strSplit(i, "=");
    if (m.size() != 2) {
      cerr << __PRETTY_FUNCTION__ << ": unable to extract name=value pair from "
           << i << endl;
      continue;
    }
    string name = trim(m[0]);
    string value = trim(m[1]);
    status = status && updatePropertyFromString(name, value);
  }
  return status;
}

G2O_END_NAMESPACE
