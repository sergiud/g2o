// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/stuff/logger.h>

#include <cassert>
#include <memory>
#include <utility>

using namespace std;

G2O_START_NAMESPACE

RobustKernelFactory* RobustKernelFactory::instance() {
  static RobustKernelFactory factoryInstance;
  return &factoryInstance;
}

void RobustKernelFactory::registerRobustKernel(
    const std::string& tag, std::unique_ptr<AbstractRobustKernelCreator> c) {
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    ngx::Log(ngx::Warning,
             "RobustKernelFactory WARNING: Overwriting robust kernel tag {}",
             tag);
    assert(0);
  }

  _creator[tag] = std::move(c);
}

void RobustKernelFactory::unregisterType(const std::string& tag) {
  auto tagPosition = _creator.find(tag);
  if (tagPosition != _creator.end()) {
    _creator.erase(tagPosition);
  }
}

std::unique_ptr<RobustKernel> RobustKernelFactory::construct(
    const std::string& tag) const {
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    return foundIt->second->construct();
  }
  return nullptr;
}

AbstractRobustKernelCreator* RobustKernelFactory::creator(
    const std::string& tag) const {
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    return foundIt->second.get();
  }
  return nullptr;
}

void RobustKernelFactory::fillKnownKernels(
    std::vector<std::string>& types) const {
  types.clear();
  types.reserve(_creator.size());

  for (const auto& c : _creator) types.push_back(c.first);
}

void RobustKernelFactory::destroy() {}

G2O_END_NAMESPACE
