// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d/se3quat.h"

#include <Eigen/Core>
#include <numbers>
#include <ostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

constexpr double Tolerance = 1e-12;

//! Tangent vector (omega, upsilon) of a rotation by the given angle.
g2o::Vector6d tangent(double angle) {
  g2o::Vector6d update;
  update.head<3>() = angle * Eigen::Vector3d(1., 2., 2.) / 3.;
  update.tail<3>() = Eigen::Vector3d(0.3, 1., -0.5);
  return update;
}

//! Exponential as the matrix exponential of the homogeneous generator.
Eigen::Matrix4d referenceExp(const g2o::Vector6d& update) {
  Eigen::Matrix4d generator = Eigen::Matrix4d::Zero();
  generator.topLeftCorner<3, 3>() =
      g2o::skew(Eigen::Vector3d(update.head<3>()));
  generator.topRightCorner<3, 1>() = update.tail<3>();
  return generator.exp();
}

class SE3QuatMap : public ::testing::TestWithParam<double> {};

}  // namespace

TEST_P(SE3QuatMap, ExpMatchesTheMatrixExponential) {
  const g2o::Vector6d update = tangent(GetParam());

  EXPECT_THAT(g2o::SE3Quat::exp(update).to_homogeneous_matrix(),
              DistanceFrom(referenceExp(update), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

TEST_P(SE3QuatMap, LogInvertsExp) {
  const g2o::Vector6d update = tangent(GetParam());

  EXPECT_THAT(g2o::SE3Quat::exp(update).log(),
              DistanceFrom(update, g2o::test::MaxAbsDifference,
                           // Near a half turn, the rotation determines its
                           // axis less precisely.
                           Le(1e-9)));
}

INSTANTIATE_TEST_SUITE_P(SE3Quat, SE3QuatMap,
                         ::testing::Values(0., 9e-6, 1e-3, 0.05, 0.5, 2.,
                                           std::numbers::pi - 1e-6));

//! A half turn has a logarithm, whose axis is that of the rotation.
TEST(SE3Quat, LogOfAHalfTurn) {
  const g2o::SE3Quat halfTurn(Eigen::Quaterniond(Eigen::AngleAxisd(
                                  std::numbers::pi, Eigen::Vector3d::UnitZ())),
                              Eigen::Vector3d(1., 2., 3.));

  const g2o::Vector6d log = halfTurn.log();

  ASSERT_TRUE(log.allFinite()) << log.transpose();
  EXPECT_THAT(g2o::SE3Quat::exp(log).to_homogeneous_matrix(),
              DistanceFrom(halfTurn.to_homogeneous_matrix(),
                           g2o::test::MaxAbsDifference, Le(1e-9)));
}
