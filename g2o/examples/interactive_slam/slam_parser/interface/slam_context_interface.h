// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef SLAM_CONTEXT_INTERFACE_H
#define SLAM_CONTEXT_INTERFACE_H

#include "slam_parser/parser/slam_context.h"
#include <g2o/slam_parser/interface/g2o_slam_parser_interface_api.h>

namespace SlamParser {

  class AbstractSlamInterface;

  class G2O_SLAM_PARSER_INTERFACE_API SlamContextInterface : public SlamContext
  {
    public:
      SlamContextInterface(AbstractSlamInterface* slam);
      ~SlamContextInterface();

      bool process(CommandNode* commandNode);

    protected:
      AbstractSlamInterface* _slam;
  };

} // namespace SlamParser

#endif
