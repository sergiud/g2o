// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <Eigen/Eigenvalues>
#include <ostream>

#include "g2o/types/icp/types_icp.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;
using ::testing::ElementsAre;

namespace {

class GicpNormal : public ::testing::TestWithParam<Eigen::Vector3d> {};

}  // namespace

/**
 * The precision of a point on a plane is the weight along the normal and a
 * small weight within the plane, whatever the direction and the length of
 * the normal it was given.
 */
TEST_P(GicpNormal, WeightsAlongAndAcrossTheNormal) {
  constexpr double InPlane = 0.01;
  constexpr double Tolerance = 1e-12;
  g2o::EdgeGICP edge;
  edge.normal0 = GetParam();

  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      edge.prec0(InPlane));

  EXPECT_THAT(solver.eigenvalues(), ElementsAre(DoubleNear(InPlane, Tolerance),
                                                DoubleNear(InPlane, Tolerance),
                                                DoubleNear(1., Tolerance)));
}

INSTANTIATE_TEST_SUITE_P(EdgeGICP, GicpNormal,
                         ::testing::Values(Eigen::Vector3d(0., 0., 1.),
                                           Eigen::Vector3d(0., 1., 0.),
                                           Eigen::Vector3d(0., 0., 2.)));
