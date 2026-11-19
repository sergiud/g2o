// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_ROBOT_DATA_H
#define G2O_ROBOT_DATA_H

#include <g2o/core/optimizable_graph.h>
#include <g2o/prefix.h>
#include <g2o/types/data/g2o_types_data_api.h>

#include <iosfwd>
#include <string>
#include <string_view>

G2O_START_NAMESPACE

/**
 * \brief data recorded by the robot
 */
class G2O_TYPES_DATA_API RobotData : public HyperGraph::Data {
 public:
  RobotData();
  virtual ~RobotData();

  double timestamp() const { return _timestamp; }
  void setTimestamp(double ts);

  double loggerTimestamp() const { return _loggerTimestamp; }
  void setLoggerTimestamp(double ts);

  const std::string& tag() const { return _tag; }
  void setTag(const std::string& tag);

  const std::string& hostname() const { return _hostname; }
  void setHostname(const std::string& hostname);

 protected:
  /**
   * Name as a single token of a line in a graph file, since a name without
   * characters leaves no token and shifts the values that follow it.
   */
  static std::string_view nameToken(const std::string& name);

  double _timestamp;        ///< timestamp when the measurement was generated
  double _loggerTimestamp;  ///< timestamp when the measurement was recorded
  std::string
      _tag;  ///< string tag (FLASER, ROBOTLASER, ODOM..) of the line in the log
  std::string _hostname;  ///< name of the computer/robot generating the data
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
