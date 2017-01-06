// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel.h>

#include <cassert>

using namespace std;

namespace g2o {

RobustKernelFactory* RobustKernelFactory::instance()
{
  static RobustKernelFactory factoryInstance;
  return &factoryInstance;
}

void RobustKernelFactory::registerRobustKernel(const std::string& tag, AbstractRobustKernelCreator* c)
{
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    cerr << "RobustKernelFactory WARNING: Overwriting robust kernel tag " << tag << endl;
    assert(0);
  }

  _creator[tag].reset(c);
}

void RobustKernelFactory::unregisterType(const std::string& tag)
{
  auto tagPosition = _creator.find(tag);
  if (tagPosition != _creator.end()) {
    tagPosition->second.release();
    _creator.erase(tagPosition);
  }
}

RobustKernel* RobustKernelFactory::construct(const std::string& tag) const
{
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    return foundIt->second->construct();
  }
  return 0;
}

AbstractRobustKernelCreator* RobustKernelFactory::creator(const std::string& tag) const
{
  auto foundIt = _creator.find(tag);
  if (foundIt != _creator.end()) {
    return foundIt->second.get();
  }
  return 0;
}

void RobustKernelFactory::fillKnownKernels(std::vector<std::string>& types) const
{
  types.clear();
  types.reserve(_creator.size());

  for (const auto& c : _creator)
    types.push_back(c.first);
}

void RobustKernelFactory::destroy()
{
}

} // end namespace
