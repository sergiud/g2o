// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <iosfwd>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/jacobian_workspace.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

//! Difference of two points, whose Jacobians are the identity and its negative.
class DifferenceEdge
    : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexPointXY,
                                 g2o::VertexPointXY> {
 public:
  void computeError() override {
    const auto* from = static_cast<const g2o::VertexPointXY*>(_vertices[0]);
    const auto* to = static_cast<const g2o::VertexPointXY*>(_vertices[1]);
    _error = to->estimate() - from->estimate() - _measurement;
  }
  bool read(std::istream&) override { return false; }
  bool write(std::ostream&) const override { return false; }
};

}  // namespace

/**
 * The numeric Jacobian of points a thousand units from the origin keeps most
 * digits, which a step too small for the spacing of doubles there loses.
 */
TEST(NumericJacobian, KeepsItsDigitsAwayFromTheOrigin) {
  constexpr double Distance = 1e3;
  constexpr double Tolerance = 1e-7;
  g2o::VertexPointXY from;
  from.setEstimate(Eigen::Vector2d(Distance, -Distance));
  g2o::VertexPointXY to;
  to.setEstimate(Eigen::Vector2d(Distance + 1., Distance));
  DifferenceEdge edge;
  edge.setVertex(0, &from);
  edge.setVertex(1, &to);
  g2o::JacobianWorkspace workspace;
  workspace.updateSize(&edge);
  workspace.allocate();

  edge.linearizeOplus(workspace);

  EXPECT_THAT(edge.jacobianOplusXj(),
              DistanceFrom(Eigen::Matrix2d::Identity(),
                           g2o::test::MaxAbsDifference, Le(Tolerance)));
  EXPECT_THAT(edge.jacobianOplusXi(),
              DistanceFrom(Eigen::Matrix2d(-Eigen::Matrix2d::Identity()),
                           g2o::test::MaxAbsDifference, Le(Tolerance)));
}
