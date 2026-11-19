// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam2d_addons/edge_line2d.h"

#include <numbers>

#include "g2o/types/slam2d_addons/vertex_line2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;
using ::testing::ElementsAre;

/**
 * The angles of two lines on either side of pi differ by their distance on
 * the circle, not by almost a full turn.
 */
TEST(EdgeLine2D, WrapsTheAngleError) {
  constexpr double Offset = 0.01;
  constexpr double Tolerance = 1e-12;
  g2o::VertexLine2D first;
  first.setEstimate(
      g2o::Line2D(Eigen::Vector2d(std::numbers::pi - Offset, 1.)));
  g2o::VertexLine2D second;
  second.setEstimate(
      g2o::Line2D(Eigen::Vector2d(-std::numbers::pi + Offset, 1.)));
  g2o::EdgeLine2D edge;
  edge.setVertex(0, &first);
  edge.setVertex(1, &second);
  edge.setMeasurement(Eigen::Vector2d::Zero());

  edge.computeError();

  EXPECT_THAT(edge.error(), ElementsAre(DoubleNear(2. * Offset, Tolerance),
                                        DoubleNear(0., Tolerance)));
}
