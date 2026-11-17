// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_COMMON_H
#define G2O_COMMON_H

#include <g2o/cli/g2o_cli_api.h>
#include <g2o/core/abi.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

class DlWrapper;
class DlSolverWrapper;

/**
 * will also look for -typeslib in (argc, argv) and load that types
 */
G2O_CLI_API void loadStandardTypes(DlWrapper& dlWrapper, int argc = 0,
                                   char** argv = nullptr);

/**
 * will also look for -solverlib in (argc, argv) and load that solver
 */
G2O_CLI_API void loadStandardSolver(DlWrapper& dlSolverWrapper, int argc = 0,
                                    char** argv = nullptr);

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
