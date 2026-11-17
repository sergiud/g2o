// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STUFF_LOGGER_H
#define G2O_STUFF_LOGGER_H

#include <g2o/stuff/g2o_stuff_api.h>
#include <g2o/stuff/streamed.h>
#include <ng-log/experimental/log.h>

#include <string_view>

namespace g2o {

//! The logging functions with which g2o reports its diagnostics.
namespace ngx = nglog::experimental;

/**
 * Sets up logging for a program of g2o, which writes the messages to the
 * standard error only, like the programs did before logging. The library
 * itself never sets up logging, which is up to the program using it.
 *
 * @param programName Name of the program, such as argv[0].
 */
G2O_STUFF_API void initializeLogging(std::string_view programName);

}  // namespace g2o

#endif
