// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "driver.h"

#include <g2o/slam_parser/g2o_slam_parser_api.h>

#include <fstream>
#include <sstream>

#include "slam_context.h"

namespace SlamParser {

Driver::Driver(class SlamContext& slamContext_)
    : trace_scanning(false), trace_parsing(false), slamContext(slamContext_) {}

bool Driver::parse_stream(std::istream& in, const std::string& sname) {
  streamname = sname;

  Scanner scanner(&in);
  scanner.set_debug(trace_scanning);
  this->lexer = &scanner;

  Parser parser(*this);
#if YYDEBUG
  parser.set_debug_level(trace_parsing);
#endif
  return (parser.parse() == 0);
}

bool Driver::parse_file(const std::string& filename) {
  std::ifstream in(filename.c_str());
  if (!in.good()) return false;
  return parse_stream(in, filename);
}

bool Driver::parse_string(const std::string& input, const std::string& sname) {
  std::istringstream iss(input);
  return parse_stream(iss, sname);
}

void Driver::error(const class location& l, const std::string& m) {
  std::cerr << l << ": " << m << std::endl;
}

void Driver::error(const std::string& m) { std::cerr << m << std::endl; }

}  // namespace SlamParser
