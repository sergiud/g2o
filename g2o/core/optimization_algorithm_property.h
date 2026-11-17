// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OPTIMIZATION_ALGORITHM_PROPERTY_H
#define G2O_OPTIMIZATION_ALGORITHM_PROPERTY_H

#include <string>

#include <g2o/core/abi.h>
#include <g2o/core/g2o_core_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

/**
 * \brief describe the properties of a solver
 */
struct G2O_CORE_API OptimizationAlgorithmProperty
{
  std::string name;           ///< name of the solver, e.g., var
  std::string desc;           ///< short description of the solver
  std::string type;           ///< type of solver, e.g., "CSparse Cholesky", "PCG"
  bool requiresMarginalize;   ///< whether the solver requires marginalization of landmarks
  int poseDim;                ///< dimension of the pose vertices (-1 if variable)
  int landmarkDim;            ///< dimension of the landmar vertices (-1 if variable)
  OptimizationAlgorithmProperty() :
    name(), desc(), type(), requiresMarginalize(false), poseDim(-1), landmarkDim(-1)
  {
  }
  OptimizationAlgorithmProperty(const std::string& name_, const std::string& desc_, const std::string& type_, bool requiresMarginalize_, int poseDim_, int landmarkDim_) :
    name(name_), desc(desc_), type(type_), requiresMarginalize(requiresMarginalize_), poseDim(poseDim_), landmarkDim(landmarkDim_)
  {
  }
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
