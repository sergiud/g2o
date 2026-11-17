// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/core/parameter.h>
#include <g2o/core/parameter_container.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/string_tools.h>

#include <iostream>

G2O_START_NAMESPACE

using namespace std;

ParameterContainer::ParameterContainer(bool isMainStorage_)
    : _isMainStorage(isMainStorage_) {}

void ParameterContainer::clear() {
  if (!_isMainStorage) return;
  for (auto& it : *this) {
    delete it.second;
  }
  BaseClass::clear();
}

ParameterContainer::~ParameterContainer() { clear(); }

bool ParameterContainer::addParameter(Parameter* p) {
  if (p->id() < 0) return false;
  auto it = find(p->id());
  if (it != end()) return false;
  insert(make_pair(p->id(), p));
  return true;
}

Parameter* ParameterContainer::getParameter(int id) {
  auto it = find(id);
  if (it == end()) return nullptr;
  return it->second;
}

const Parameter* ParameterContainer::getParameter(int id) const {
  auto it = find(id);
  if (it == end()) return nullptr;
  return it->second;
}

Parameter* ParameterContainer::detachParameter(int id) {
  auto it = find(id);
  if (it == end()) return nullptr;
  Parameter* p = it->second;
  erase(it);
  return p;
}

bool ParameterContainer::write(std::ostream& os) const {
  Factory* factory = Factory::instance();
  for (const auto& it : *this) {
    os << factory->tag(it.second) << " ";
    os << it.second->id() << " ";
    it.second->write(os);
    os << endl;
  }
  return true;
}

bool ParameterContainer::read(
    std::istream& is,
    const std::map<std::string, std::string>* _renamedTypesLookup) {
  stringstream currentLine;
  string token;

  Factory* factory = Factory::instance();
  HyperGraph::GraphElemBitset elemBitset;
  elemBitset[HyperGraph::HGET_PARAMETER] = 1;

  while (1) {
    int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    currentLine >> token;
    if (bytesRead == 0 || token.empty() || token[0] == '#') continue;
    if ((_renamedTypesLookup != nullptr) && !_renamedTypesLookup->empty()) {
      auto foundIt = _renamedTypesLookup->find(token);
      if (foundIt != _renamedTypesLookup->end()) {
        token = foundIt->second;
      }
    }

    HyperGraph::HyperGraphElement* element =
        factory->construct(token, elemBitset);
    if (element == nullptr)  // not a parameter or otherwise unknown tag
      continue;
    assert(element->elementType() == HyperGraph::HGET_PARAMETER &&
           "Should be a param");

    Parameter* p = dynamic_cast<Parameter*>(element);
    int pid;
    currentLine >> pid;
    p->setId(pid);
    bool r = p->read(currentLine);
    if (!r) {
      ngx::Log(ngx::Error, "Error reading data {} for parameter {}", token,
               pid);
      delete p;
    } else {
      if (!addParameter(p)) {
        ngx::Log(ngx::Warning, "Parameter of type:{} id:{} already defined",
                 token, pid);
      }
    }
  }  // while read line

  return true;
}

G2O_END_NAMESPACE
