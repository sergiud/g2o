// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef PARSER_INTERFACE_H
#define PARSER_INTERFACE_H

#include "abstract_slam_interface.h"
#include <g2o/slam_parser/interface/g2o_slam_parser_interface_api.h>

#include <iosfwd>
#include <sstream>

namespace SlamParser {

  class SlamContextInterface;
  class Driver;

  /**
   * \brief top-level interface to the parser
   */
  class G2O_SLAM_PARSER_INTERFACE_API ParserInterface
  {
    public:
      /**
       * construct a parser and use the given AbstractSlamInterface to communicate with the SLAM algorithm.
       */
      ParserInterface(AbstractSlamInterface* slamInterface);
      virtual ~ParserInterface();

      /**
       * parse a single command and forward to the SLAM engine
       */ 
      bool parseCommand(std::istream& input);

    protected:
      SlamContextInterface* _slamContextInterface;
      Driver* _driver;
      std::stringstream _buffer;
  };

} // namespace SlamParser

#endif
