// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "gtest/gtest.h"

using namespace std;

TEST(Slam3D, OrthogonalMatrix) {
  Eigen::Matrix3d shouldBeIdentity;
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rot =
      (Eigen::Matrix3d)Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ());
  rot =
      rot * (Eigen::Matrix3d)Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX());

  shouldBeIdentity = R * R.transpose();
  double initialDifference =
      (shouldBeIdentity - Eigen::Matrix3d::Identity()).array().abs().maxCoeff();
  EXPECT_DOUBLE_EQ(0., initialDifference);

  // introduce numerical inaccuracies
  for (int i = 0; i < 10000; ++i) R = R * rot;
  shouldBeIdentity = R * R.transpose();
  double afterMultDifference =
      (shouldBeIdentity - Eigen::Matrix3d::Identity()).array().abs().maxCoeff();
  EXPECT_GE(afterMultDifference, initialDifference);

  double inaccurateDet = R.determinant();
  Eigen::Matrix3d approxSolution = R;
  g2o::internal::approximateNearestOrthogonalMatrix(approxSolution);
  g2o::internal::nearestOrthogonalMatrix(R);

  EXPECT_LE(std::abs(R.determinant() - 1.), std::abs(inaccurateDet - 1.));
  EXPECT_NEAR(1.0, R.determinant(), 1e-7);
  shouldBeIdentity = R * R.transpose();
  double nearestDifference =
      (shouldBeIdentity - Eigen::Matrix3d::Identity()).array().abs().maxCoeff();
  EXPECT_NEAR(0., nearestDifference, 1e-7);

  // norm of the comluns
  for (int i = 0; i < 3; ++i) EXPECT_NEAR(1.0, R.col(i).norm(), 1e-7);

  shouldBeIdentity = approxSolution * approxSolution.transpose();
  double approxNearestDifference =
      (shouldBeIdentity - Eigen::Matrix3d::Identity()).array().abs().maxCoeff();
  EXPECT_NEAR(0., approxNearestDifference, 1e-6);
  EXPECT_LE(std::abs(R.determinant() - 1.),
            1e-15 + std::abs(approxSolution.determinant() - 1.));
  EXPECT_NEAR(1.0, approxSolution.determinant(), 1e-6);
  for (int i = 0; i < 3; ++i)
    EXPECT_NEAR(1.0, approxSolution.col(i).norm(), 1e-6);
}
