// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d_addons/line3d.h"

#include <ostream>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

constexpr double Tolerance = 1e-9;

//! Line through the given point in the given direction.
g2o::Line3D line(const Eigen::Vector3d& point,
                 const Eigen::Vector3d& direction) {
  g2o::Vector6d cartesian;
  cartesian << point, direction;
  return g2o::Line3D::fromCartesian(cartesian);
}

//! Point of a line and the angle of a step of its distance to the origin.
struct LineStep {
  Eigen::Vector3d point;
  double angle;
};

void PrintTo(const LineStep& step, std::ostream* os) {
  *os << "point " << step.point.transpose() << " angle " << step.angle;
}

class LineIncrement : public ::testing::TestWithParam<LineStep> {};

}  // namespace

/**
 * A small step of a line is recovered as the difference to the line it
 * moved from, also for a line through the origin, whose moment is zero.
 */
TEST_P(LineIncrement, OminusInvertsOplus) {
  const g2o::Line3D start =
      line(GetParam().point, Eigen::Vector3d(0.6, 0., 0.8));
  const Eigen::Vector4d step(0.01, -0.02, 0.015, GetParam().angle);
  g2o::Line3D moved = start;

  moved.oplus(step);

  EXPECT_THAT(g2o::Line3D(start).ominus(moved),
              DistanceFrom(step, g2o::test::MaxAbsDifference, Le(Tolerance)));
}

// The orthonormal form keeps the distance to the origin positive, so a line
// through the origin moves away from it only with a negative angle.
INSTANTIATE_TEST_SUITE_P(
    Line3D, LineIncrement,
    ::testing::Values(LineStep{Eigen::Vector3d(1., 2., 0.), 0.03},
                      LineStep{Eigen::Vector3d::Zero(), -0.03}));

//! The default line passes through the origin.
TEST(Line3D, DefaultLineMoves) {
  g2o::Line3D moved;

  moved.oplus(Eigen::Vector4d(0.01, 0., 0., 0.));

  EXPECT_TRUE(moved.allFinite()) << moved.transpose();
}
