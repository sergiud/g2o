// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/data/data_queue.h>
#include <g2o/types/data/robot_data.h>

#include <iterator>
#include <memory>
#include <utility>

G2O_START_NAMESPACE

DataQueue::DataQueue() {}

DataQueue::~DataQueue() = default;

RobotData* DataQueue::findClosestData(double timestamp) const {
  if (_buffer.empty()) return nullptr;
  // The first entry after the time, which the newest entry has none of.
  auto ub = _buffer.upper_bound(timestamp);
  if (ub == _buffer.end()) return std::prev(ub)->second.get();
  if (ub == _buffer.begin()) return ub->second.get();
  auto lb = std::prev(ub);
  if (fabs(lb->first - timestamp) < fabs(ub->first - timestamp))
    return lb->second.get();
  return ub->second.get();
}

RobotData* DataQueue::before(double timestamp) const {
  if (_buffer.empty() || _buffer.begin()->first > timestamp) return nullptr;
  auto lb = _buffer.upper_bound(timestamp);
  --lb;  // now it's the lower bound
  return lb->second.get();
}

RobotData* DataQueue::after(double timestamp) const {
  if (_buffer.empty() || _buffer.rbegin()->first < timestamp) return nullptr;
  auto ub = _buffer.upper_bound(timestamp);
  if (ub == _buffer.end()) return nullptr;
  return ub->second.get();
}

void DataQueue::add(std::unique_ptr<RobotData> rd) {
  const double timestamp = rd->timestamp();
  _buffer[timestamp] = std::move(rd);
}

G2O_END_NAMESPACE
