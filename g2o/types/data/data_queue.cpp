// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/data/data_queue.h>
#include <g2o/types/data/robot_data.h>

G2O_START_NAMESPACE

DataQueue::DataQueue() {}

DataQueue::~DataQueue() = default;

RobotData* DataQueue::findClosestData(double timestamp) const {
  if (_buffer.rbegin()->first < timestamp) return _buffer.rbegin()->second;
  if (_buffer.begin()->first > timestamp) return _buffer.begin()->second;

  auto ub = _buffer.upper_bound(timestamp);
  auto lb = ub;
  --lb;
  if (fabs(lb->first - timestamp) < fabs(ub->first - timestamp))
    return lb->second;
  else
    return ub->second;
}

RobotData* DataQueue::before(double timestamp) const {
  if (_buffer.empty() || _buffer.begin()->first > timestamp) return nullptr;
  auto lb = _buffer.upper_bound(timestamp);
  --lb;  // now it's the lower bound
  return lb->second;
}

RobotData* DataQueue::after(double timestamp) const {
  if (_buffer.empty() || _buffer.rbegin()->first < timestamp) return nullptr;
  auto ub = _buffer.upper_bound(timestamp);
  if (ub == _buffer.end()) return nullptr;
  return ub->second;
}

void DataQueue::add(RobotData* rd) { _buffer[rd->timestamp()] = rd; }

G2O_END_NAMESPACE
