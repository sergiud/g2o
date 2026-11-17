// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/logger.h>

#include <iostream>

#include "example_slam_interface.h"
#include "slam_parser/interface/parser_interface.h"
using namespace std;

int main() {
  g2o::initializeLogging("test_slam_interface");
  ExampleSlamInterface slamInterface;
  SlamParser::ParserInterface parserInterface(&slamInterface);

  while (parserInterface.parseCommand(cin)) {
    // do something additional if needed
  }

  return 0;
}
