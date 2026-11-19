// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/sim3/sim3.h"

#include <Eigen/Core>
#include <ostream>
#include <unsupported/Eigen/MatrixFunctions>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

//! Rotation angle and logarithm of the scale of a similarity.
struct Motion {
  double angle;
  double logScale;
};

void PrintTo(const Motion& motion, std::ostream* os) {
  *os << "angle " << motion.angle << ", log scale " << motion.logScale;
}

constexpr double Tolerance = 1e-12;

//! Tangent vector (omega, upsilon, sigma) of the given motion.
g2o::Vector7d tangent(const Motion& motion) {
  g2o::Vector7d update;
  update.head<3>() = motion.angle * Eigen::Vector3d(1., 2., 2.) / 3.;
  update.segment<3>(3) = Eigen::Vector3d(0.3, 1., -0.5);
  update[6] = motion.logScale;
  return update;
}

/**
 * Exponential of the tangent vector as the matrix exponential of the
 * homogeneous generator, which serves as the reference.
 */
Eigen::Matrix4d referenceExp(const g2o::Vector7d& update) {
  Eigen::Matrix4d generator = Eigen::Matrix4d::Zero();
  generator.topLeftCorner<3, 3>() =
      g2o::skew(Eigen::Vector3d(update.head<3>())) +
      update[6] * Eigen::Matrix3d::Identity();
  generator.topRightCorner<3, 1>() = update.segment<3>(3);
  return generator.exp();
}

Eigen::Matrix4d homogeneous(const g2o::Sim3& similarity) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.topLeftCorner<3, 3>() =
      similarity.scale() * similarity.rotation().toRotationMatrix();
  matrix.topRightCorner<3, 1>() = similarity.translation();
  return matrix;
}

class Sim3Map : public ::testing::TestWithParam<Motion> {};

}  // namespace

TEST_P(Sim3Map, ExpMatchesTheMatrixExponential) {
  const g2o::Vector7d update = tangent(GetParam());

  EXPECT_THAT(homogeneous(g2o::Sim3(update)),
              DistanceFrom(referenceExp(update), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

TEST_P(Sim3Map, LogInvertsExp) {
  const g2o::Vector7d update = tangent(GetParam());

  EXPECT_THAT(g2o::Sim3(update).log(),
              DistanceFrom(update, g2o::test::MaxAbsDifference, Le(Tolerance)));
}

INSTANTIATE_TEST_SUITE_P(Sim3, Sim3Map,
                         ::testing::Values(Motion{0., 0.}, Motion{9e-6, 0.},
                                           Motion{1e-3, 0.}, Motion{0.5, 0.},
                                           Motion{0., 1e-4}, Motion{9e-6, 1e-4},
                                           Motion{1e-3, 1e-2}, Motion{0.5, 0.1},
                                           Motion{2., -0.7}, Motion{1e-6, -0.7},
                                           Motion{1e-3, 3.}, Motion{1e-3, -3.},
                                           Motion{0.9, 3.}, Motion{2., 3.}));
