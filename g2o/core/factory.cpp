// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2018 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/cache.h>
#include <g2o/core/creators.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/parameter.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/logger.h>

#include <cassert>
#include <iostream>
#include <memory>
#include <typeinfo>
#include <utility>

using namespace std;

G2O_START_NAMESPACE

Factory::~Factory() {
#ifdef G2O_DEBUG_FACTORY
  ngx::Log("# Factory destroying {}", (void*)this);
#endif
}

Factory* Factory::instance() {
  static Factory factory;
#ifdef G2O_DEBUG_FACTORY
  ngx::Log("# Factory allocated {}", (void*)&factory);
#endif

  return &factory;
}

void Factory::registerType(
    const std::string& tag,
    std::unique_ptr<AbstractHyperGraphElementCreator> c) {
  CreatorMap::const_iterator foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    ngx::Log(ngx::Warning, "FACTORY WARNING: Overwriting Vertex tag {}", tag);
    assert(0);
  }
  TagLookup::const_iterator tagIt = _tagLookup.find(c->name());
  if (tagIt != _tagLookup.end()) {
    ngx::Log(ngx::Warning,
             "FACTORY WARNING: Registering same class for two tags {}",
             c->name());
    assert(0);
  }

  auto ci = std::make_unique<CreatorInformation>();
  AbstractHyperGraphElementCreator* creator = c.get();
  ci->creator = std::move(c);

#ifdef G2O_DEBUG_FACTORY
  ngx::Log("# Factory {} constructing type {}", static_cast<void*>(this), tag);
#endif
  // construct an element once to figure out its type
  std::unique_ptr<HyperGraph::HyperGraphElement> element(creator->construct());
  ci->elementTypeBit = element->elementType();

#ifdef G2O_DEBUG_FACTORY
  const char* elementType = "";
  switch (element->elementType()) {
    case HyperGraph::HGET_VERTEX:
      elementType = "Vertex";
      break;
    case HyperGraph::HGET_EDGE:
      elementType = "Edge";
      break;
    case HyperGraph::HGET_PARAMETER:
      elementType = "Parameter";
      break;
    case HyperGraph::HGET_CACHE:
      elementType = "Cache";
      break;
    case HyperGraph::HGET_DATA:
      elementType = "Data";
      break;
    default:
      assert(0 && "Unknown element type occured, fix elementTypes");
      break;
  }
  ngx::Log("# Factory {} registering {} {} -> {}", static_cast<void*>(this),
           tag, static_cast<void*>(creator), elementType);
#endif

  _creator.emplace(tag, std::move(ci));
  _tagLookup[creator->name()] = tag;
}

void Factory::unregisterType(const std::string& tag) {
  // Look for the tag
  auto tagPosition = _creator.find(tag);

  if (tagPosition != _creator.end()) {
    auto& c = tagPosition->second->creator;

    // If we found it, remove the creator from the tag lookup map
    auto classPosition = _tagLookup.find(c->name());
    if (classPosition != _tagLookup.end()) {
      _tagLookup.erase(classPosition);
    }
    _creator.erase(tagPosition);
  }
}

std::unique_ptr<HyperGraph::HyperGraphElement> Factory::construct(
    const std::string& tag) const {
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    // cerr << "tag " << tag << " -> " << (void*) foundIt->second->creator << "
    // " << foundIt->second->creator->name() << endl;
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

const std::string& Factory::tag(const HyperGraph::HyperGraphElement* e) const {
  static std::string emptyStr("");
  auto foundIt = _tagLookup.find(typeid(*e).name());
  if (foundIt != _tagLookup.end()) return foundIt->second;
  return emptyStr;
}

void Factory::fillKnownTypes(std::vector<std::string>& types) const {
  types.clear();
  for (const auto& it : _creator) types.push_back(it.first);
}

bool Factory::knowsTag(const std::string& tag, int* elementType) const {
  auto foundIt = _creator.find(tag);
  if (foundIt == _creator.end()) {
    if (elementType != nullptr) *elementType = -1;
    return false;
  }
  if (elementType != nullptr) *elementType = foundIt->second->elementTypeBit;
  return true;
}

void Factory::destroy() {}

void Factory::printRegisteredTypes(std::ostream& os, bool comment) const {
  if (comment) os << "# ";
  os << "types:" << endl;
  for (const auto& it : _creator) {
    if (comment) os << "#";
    os << "\t" << it.first << endl;
  }
}

std::unique_ptr<HyperGraph::HyperGraphElement> Factory::construct(
    const std::string& tag,
    const HyperGraph::GraphElemBitset& elemsToConstruct) const {
  if (elemsToConstruct.none()) {
    return construct(tag);
  }
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end() && foundIt->second->elementTypeBit >= 0 &&
      elemsToConstruct.test(foundIt->second->elementTypeBit)) {
    return foundIt->second->creator->construct();
  }
  return nullptr;
}

G2O_END_NAMESPACE
