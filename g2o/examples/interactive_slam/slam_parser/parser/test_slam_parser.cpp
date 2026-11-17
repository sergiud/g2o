// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <iostream>

#include "commands.h"
#include "driver.h"
#include "slam_context.h"
using namespace std;

int main() {
  SlamParser::SlamContext slamContext;
  SlamParser::Driver driver(slamContext);
  driver.trace_parsing = true;
  driver.trace_scanning = true;

  bool parseStatus = driver.parse_stream(cin);
  if (!parseStatus) return 1;
}
