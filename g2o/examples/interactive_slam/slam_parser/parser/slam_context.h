// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef SLAM_CONTEXT_H
#define SLAM_CONTEXT_H

#include <g2o/slam_parser/g2o_slam_parser_api.h>

#include <vector>

#include "scanner.h"

namespace SlamParser {

class CommandNode;

class G2O_SLAM_PARSER_API SlamContext {
 public:
  SlamContext();
  virtual ~SlamContext();

  virtual bool process(CommandNode* commandNode);
};

}  // namespace SlamParser

#endif
