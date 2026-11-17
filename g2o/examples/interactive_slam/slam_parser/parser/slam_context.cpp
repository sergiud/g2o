// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "slam_context.h"

#include <iostream>

#include "commands.h"
using namespace std;

namespace SlamParser {

SlamContext::SlamContext() = default;

SlamContext::~SlamContext() = default;

bool SlamContext::process(CommandNode* commandNode) {
  cerr << "SlamContext::process -> ";
  switch (commandNode->commandType()) {
    case CT_ADD_NODE:
      cerr << "ADD NODE" << endl;
      break;
    case CT_ADD_EDGE:
      cerr << "ADD EDGE" << endl;
      break;
    case CT_SOLVE_STATE:
      cerr << "SOLVE STATE" << endl;
      break;
    case CT_QUERY_STATE:
      cerr << "QUERY STATE" << endl;
      break;
    case CT_FIX:
      cerr << "FIX NODE" << endl;
      break;
  }
  return true;
}

}  // namespace SlamParser
