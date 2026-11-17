// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include "gtest/gtest.h"

#include "g2o/types/slam2d/se2.h"

using namespace std;
using namespace g2o;

TEST(MappingsSlam2D, SE2)
{
  SE2 s1(0., 1., 2.);
  EXPECT_DOUBLE_EQ(0., s1[0]);
  EXPECT_DOUBLE_EQ(1., s1[1]);
  EXPECT_DOUBLE_EQ(2., s1[2]);
}
