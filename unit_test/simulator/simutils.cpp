// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/apps/g2o_simulator/simutils.h"

#include <Eigen/Core>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

/**
 * A segment that leaves the circle ends where it crosses the circle, past
 * its start rather than behind it.
 */
TEST(ClipSegmentCircle, ClipsTheEndOutside) {
  Eigen::Vector2d p1(0., 0.);
  Eigen::Vector2d p2(2., 0.);

  EXPECT_EQ(g2o::clipSegmentCircle(p1, p2, 1.), 1);

  EXPECT_THAT(p1, ElementsAre(0., 0.));
  EXPECT_THAT(p2, ElementsAre(1., 0.));
}

//! A segment through the circle keeps the part between its crossings.
TEST(ClipSegmentCircle, ClipsBothEnds) {
  Eigen::Vector2d p1(-2., 0.);
  Eigen::Vector2d p2(2., 0.);

  EXPECT_EQ(g2o::clipSegmentCircle(p1, p2, 1.), 3);

  EXPECT_THAT(p1, ElementsAre(-1., 0.));
  EXPECT_THAT(p2, ElementsAre(1., 0.));
}
