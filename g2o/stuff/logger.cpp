// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/logger.h>
#include <ng-log/experimental/setup.h>

namespace g2o {

void initializeLogging(std::string_view programName) {
  ngx::Initialize(programName);
  ngx::LogOnlyToStderr();
}

}  // namespace g2o
