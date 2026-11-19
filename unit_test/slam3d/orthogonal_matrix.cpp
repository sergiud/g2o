// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <span>

#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::DoubleNear;
using ::testing::Each;
using ::testing::Gt;
using ::testing::Le;
using ::testing::Lt;

using namespace std;

TEST(Slam3D, OrthogonalMatrix) {
  Eigen::Matrix3d shouldBeIdentity;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rot =
      (Eigen::Matrix3d)Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ());
  rot =
      rot * (Eigen::Matrix3d)Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX());

  shouldBeIdentity = R * R.transpose();
  const double initialDifference = g2o::test::MaxAbsDifference(
      shouldBeIdentity, Eigen::Matrix3d::Identity());
  EXPECT_DOUBLE_EQ(0., initialDifference);

  // introduce numerical inaccuracies, which the rest of the test removes
  for (int i = 0; i < 10000; ++i) R = R * rot;
  shouldBeIdentity = R * R.transpose();
  ASSERT_THAT(shouldBeIdentity,
              DistanceFrom(Eigen::Matrix3d::Identity(),
                           g2o::test::MaxAbsDifference, Gt(0.)));

  double inaccurateDet = R.determinant();
  Eigen::Matrix3d approxSolution = R;
  g2o::internal::approximateNearestOrthogonalMatrix(approxSolution);
  g2o::internal::nearestOrthogonalMatrix(R);

  EXPECT_THAT(R.determinant(),
              DistanceFrom(1., Le(std::abs(inaccurateDet - 1.))));
  EXPECT_NEAR(1.0, R.determinant(), 1e-7);
  shouldBeIdentity = R * R.transpose();
  EXPECT_THAT(shouldBeIdentity,
              DistanceFrom(Eigen::Matrix3d::Identity(),
                           g2o::test::MaxAbsDifference, Le(1e-7)));

  // norm of the columns
  const Eigen::RowVector3d norms = R.colwise().norm();
  EXPECT_THAT(std::span(norms.data(), 3), Each(DoubleNear(1.0, 1e-7)));

  shouldBeIdentity = approxSolution * approxSolution.transpose();
  EXPECT_THAT(shouldBeIdentity,
              DistanceFrom(Eigen::Matrix3d::Identity(),
                           g2o::test::MaxAbsDifference, Le(1e-6)));
  EXPECT_THAT(R.determinant(),
              DistanceFrom(
                  1., Le(1e-15 + std::abs(approxSolution.determinant() - 1.))));
  EXPECT_NEAR(1.0, approxSolution.determinant(), 1e-6);
  const Eigen::RowVector3d approxNorms = approxSolution.colwise().norm();
  EXPECT_THAT(std::span(approxNorms.data(), 3), Each(DoubleNear(1.0, 1e-6)));
}

/**
 * One step of the approximation reduces the deviation of a perturbed rotation
 * from an orthogonal matrix quadratically.
 */
TEST(Slam3D, ApproximateNearestOrthogonalMatrixReducesDeviation) {
  constexpr double perturbation = 1e-3;
  // The quadratic convergence leaves at most this multiple of the squared
  // initial deviation.
  constexpr double convergenceFactor = 10.;
  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(0.4, Eigen::Vector3d(1., 2., 3.).normalized())
          .toRotationMatrix();
  Eigen::Matrix3d noise;
  noise << 0.3, -0.7, 0.2, 0.9, 0.1, -0.4, -0.5, 0.6, 0.8;
  Eigen::Matrix3d R = rotation + perturbation * noise;
  const auto product = [](const Eigen::Matrix3d& m) -> Eigen::Matrix3d {
    return m * m.transpose();
  };
  const double initialDeviation =
      g2o::test::MaxAbsDifference(product(R), Eigen::Matrix3d::Identity());
  ASSERT_GT(initialDeviation, perturbation / convergenceFactor);

  g2o::internal::approximateNearestOrthogonalMatrix(R);

  EXPECT_THAT(product(R),
              DistanceFrom(
                  Eigen::Matrix3d::Identity(), g2o::test::MaxAbsDifference,
                  Lt(convergenceFactor * initialDeviation * initialDeviation)));
}
