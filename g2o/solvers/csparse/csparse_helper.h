// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: LGPL-3.0-or-later

#ifndef G2O_CSPARSE_HELPER_H
#define G2O_CSPARSE_HELPER_H

#ifndef NCOMPLEX
#define NCOMPLEX
#endif
#include <cs.h>
#include <g2o/core/abi.h>
#include <g2o/prefix.h>
#include <g2o/solvers/csparse/g2o_csparse_extension_api.h>

G2O_START_NAMESPACE

namespace csparse_extension {

/**
 * write the sparse matrix to a file loadable with ocatve
 */
G2O_CSPARSE_EXTENSION_API bool writeCs2Octave(const char* filename, const cs* A,
                                              bool upperTriangular = true);

// our extensions to csparse
G2O_CSPARSE_EXTENSION_API csn* cs_chol_workspace(const cs* A, const css* S,
                                                 int* cin, double* xin);
G2O_CSPARSE_EXTENSION_API int cs_cholsolsymb(const cs* A, double* b,
                                             const css* S, double* workspace,
                                             int* work);

}  // namespace csparse_extension
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
