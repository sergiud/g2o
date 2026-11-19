// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/data/data_queue.h"

#include <memory>

#include "g2o/types/data/robot_laser.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::IsNull;
using ::testing::NotNull;

namespace {

//! Queue of laser scans at the times 1, 2 and 4.
class ClosestData : public ::testing::Test {
 protected:
  void SetUp() override {
    for (const double timestamp : {1., 2., 4.}) {
      auto laser = std::make_unique<g2o::RobotLaser>();
      laser->setTimestamp(timestamp);
      _queue.add(std::move(laser));
    }
  }

  double closest(double timestamp) const {
    const g2o::RobotData* data = _queue.findClosestData(timestamp);
    EXPECT_THAT(data, NotNull());
    return data == nullptr ? -1. : data->timestamp();
  }

  g2o::DataQueue _queue;
};

}  // namespace

TEST_F(ClosestData, MatchesTheOldestAndTheNewest) {
  EXPECT_EQ(closest(1.), 1.);
  EXPECT_EQ(closest(4.), 4.);
}

TEST_F(ClosestData, ClampsToTheEnds) {
  EXPECT_EQ(closest(0.), 1.);
  EXPECT_EQ(closest(5.), 4.);
}

TEST_F(ClosestData, PicksTheCloserNeighbor) {
  EXPECT_EQ(closest(2.9), 2.);
  EXPECT_EQ(closest(3.1), 4.);
}

//! An empty queue holds no data close to any time.
TEST(DataQueue, FindsNothingWhenEmpty) {
  const g2o::DataQueue queue;

  EXPECT_THAT(queue.findClosestData(1.), IsNull());
}
