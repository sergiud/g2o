// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <sstream>

#include "g2o/types/slam3d_addons/edge_se3_line.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

//! The information comes as its upper triangle, whose values all count.
TEST(EdgeSE3Line3D, ReadsTheWholeInformation) {
  g2o::EdgeSE3Line3D edge;
  std::istringstream is("0  0 0 1 1 0 0  1 0.1 0.2 0.3  2 0.4 0.5  3 0.6  4");

  ASSERT_TRUE(edge.read(is));

  const Eigen::Matrix4d& information = edge.information();
  EXPECT_THAT(information.reshaped(),
              ElementsAre(1., 0.1, 0.2, 0.3, 0.1, 2., 0.4, 0.5, 0.2, 0.4, 3.,
                          0.6, 0.3, 0.5, 0.6, 4.));
}
