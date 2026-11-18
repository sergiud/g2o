// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "parser_interface.h"

#include <memory>

#include "slam_context_interface.h"
#include "slam_parser/parser/driver.h"

namespace SlamParser {

ParserInterface::ParserInterface(AbstractSlamInterface* slamInterface) {
  _slamContextInterface = std::make_unique<SlamContextInterface>(slamInterface);
  _driver = std::make_unique<Driver>(*_slamContextInterface);
  //_driver->trace_parsing = true;
  //_driver->trace_scanning = true;
}

// The driver refers to the context, so it is destroyed first.
ParserInterface::~ParserInterface() = default;

bool ParserInterface::parseCommand(std::istream& input) {
  if (input.eof()) return false;
  _buffer.str("");
  _buffer.clear();
  input.get(*_buffer.rdbuf(), ';');
  if (!input.eof())  // get the ';'
    _buffer << (char)input.get();
  _driver->parse_stream(_buffer);
  return true;
}

}  // namespace SlamParser
