// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "slam_context_interface.h"

#include <g2o/stuff/logger.h>

#include <vector>

#include "abstract_slam_interface.h"
#include "slam_parser/parser/commands.h"
using namespace std;

namespace SlamParser {

SlamContextInterface::SlamContextInterface(AbstractSlamInterface* slam)
    : SlamContext(), _slam(slam) {}

SlamContextInterface::~SlamContextInterface() = default;

bool SlamContextInterface::process(CommandNode* commandNode) {
  if (_slam == nullptr) {
    return true;
  }
  switch (commandNode->commandType()) {
    case CT_ADD_NODE: {
      AddNode* c = dynamic_cast<AddNode*>(commandNode);
      return _slam->addNode(c->tag(), c->id(), c->dimension(), c->values());
    }
    case CT_ADD_EDGE: {
      AddEdge* c = dynamic_cast<AddEdge*>(commandNode);
      return _slam->addEdge(c->tag(), c->id(), c->dimension(), c->id1(),
                            c->id2(), c->values(), c->information());
    }
    case CT_SOLVE_STATE: {
      // SolveState* c = static_cast<SolveState*>(commandNode);
      return _slam->solveState();
    }
    case CT_QUERY_STATE: {
      QueryState* c = dynamic_cast<QueryState*>(commandNode);
      return _slam->queryState(c->ids());
    }
    case CT_FIX: {
      FixNode* c = dynamic_cast<FixNode*>(commandNode);
      return _slam->fixNode(c->ids());
    }
  }
  g2o::ngx::Log(g2o::ngx::Error,
                "SlamContextInterface::process: Unknown command type");
  return false;
}

}  // namespace SlamParser
