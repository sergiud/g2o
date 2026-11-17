// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SPARSE_HELPER_H
#define G2O_SPARSE_HELPER_H

#include <g2o/core/abi.h>
#include <g2o/stuff/g2o_stuff_api.h>

#include <string>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * write an array to a file, debugging
   */
  G2O_STUFF_API bool writeVector(const std::string& filename, const double*v, int n);

  /**
   * write a CCS matrix given by pointer to column, row, and values
   */
  G2O_STUFF_API bool writeCCSMatrix(const std::string& filename, int rows, int cols, const int* p, const int* i, const double* v, bool upperTriangleSymmetric = true);

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
