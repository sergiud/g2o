// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "slam_context.h"

#include <g2o/stuff/logger.h>

#include "commands.h"
using namespace std;

namespace SlamParser {

SlamContext::SlamContext() = default;

SlamContext::~SlamContext() = default;

bool SlamContext::process(CommandNode* commandNode) {
  g2o::ngx::Log("SlamContext::process");
  switch (commandNode->commandType()) {
    case CT_ADD_NODE:
      g2o::ngx::Log("ADD NODE");
      break;
    case CT_ADD_EDGE:
      g2o::ngx::Log("ADD EDGE");
      break;
    case CT_SOLVE_STATE:
      g2o::ngx::Log("SOLVE STATE");
      break;
    case CT_QUERY_STATE:
      g2o::ngx::Log("QUERY STATE");
      break;
    case CT_FIX:
      g2o::ngx::Log("FIX NODE");
      break;
  }
  return true;
}

}  // namespace SlamParser
