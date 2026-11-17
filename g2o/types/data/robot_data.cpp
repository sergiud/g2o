// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/data/robot_data.h>

G2O_START_NAMESPACE

  RobotData::RobotData() :
    HyperGraph::Data(),
    _timestamp(-1.), _loggerTimestamp(-1.)
  {
  }

  RobotData::~RobotData()
  = default;

  void RobotData::setTimestamp(double ts)
  {
    _timestamp = ts;
  }

  void RobotData::setLoggerTimestamp(double ts)
  {
    _loggerTimestamp = ts;
  }

  void RobotData::setTag(const std::string& tag)
  {
    _tag = tag;
  }

  void RobotData::setHostname(const std::string& hostname)
  {
    _hostname = hostname;
  }

G2O_END_NAMESPACE
