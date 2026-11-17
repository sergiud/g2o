// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/optimization_algorithm.h>

using namespace std;

G2O_START_NAMESPACE

OptimizationAlgorithm::OptimizationAlgorithm() = default;

OptimizationAlgorithm::~OptimizationAlgorithm() = default;

void OptimizationAlgorithm::printProperties(std::ostream& os) const
{
  os << "------------- Algorithm Properties -------------"  << endl;
  for (const auto & _propertie : _properties) {
    BaseProperty* p = _propertie.second;
    os << _propertie.first << "\t" << p->toString() << endl;
  }
  os << "------------------------------------------------" << endl;
}

bool OptimizationAlgorithm::updatePropertiesFromString(const std::string& propString)
{
  return _properties.updateMapFromString(propString);
}

void OptimizationAlgorithm::setOptimizer(SparseOptimizer* optimizer)
{
  _optimizer = optimizer;
}

G2O_END_NAMESPACE
