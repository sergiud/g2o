// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "gtest/gtest.h"

using namespace std;

TEST(MappingsSlam3D, EulerConversion) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);
  Eigen::Vector3d eulerAnglesFromMatrix = g2o::internal::toEuler(m1);
  for (int i = 0; i < 3; ++i)
    EXPECT_DOUBLE_EQ(eulerAngles(i), eulerAnglesFromMatrix(i));
}

TEST(MappingsSlam3D, QuaternionConversion) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);

  Eigen::Vector3d q = g2o::internal::toCompactQuaternion(m1);
  Eigen::Matrix3d m2 = g2o::internal::fromCompactQuaternion(q);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c) EXPECT_DOUBLE_EQ(m1(r, c), m2(r, c));
}

TEST(MappingsSlam3D, ET) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix<double, 6, 1> et;
  Eigen::Vector3d t(1., 2., 3.);
  et.block<3, 1>(0, 0) = t;
  et.block<3, 1>(3, 0) = eulerAngles;
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);

  Eigen::Isometry3d i1 = g2o::internal::fromVectorET(et);
  for (int r = 0; r < 3; ++r) EXPECT_EQ(t(r), i1.translation()(r));

  EXPECT_NEAR(0., (i1.linear() - m1).array().abs().maxCoeff(), 1e-6);

  Eigen::Matrix<double, 6, 1> et2 = g2o::internal::toVectorET(i1);
  EXPECT_NEAR(0., (et - et2).array().abs().maxCoeff(), 1e-6);
}

TEST(MappingsSlam3D, MQT) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix<double, 6, 1> et;
  Eigen::Vector3d t(1., 2., 3.);
  et.block<3, 1>(0, 0) = t;
  et.block<3, 1>(3, 0) = eulerAngles;
  Eigen::Isometry3d i1 = g2o::internal::fromVectorET(et);

  Eigen::Matrix<double, 6, 1> qt1 = g2o::internal::toVectorMQT(i1);

  Eigen::Isometry3d i2 = g2o::internal::fromVectorMQT(qt1);
  EXPECT_NEAR(0., (i1.linear() - i2.linear()).array().abs().maxCoeff(), 1e-6);
  EXPECT_NEAR(
      0., (i1.translation() - i2.translation()).array().abs().maxCoeff(), 1e-6);
}

TEST(MappingsSlam3D, QT) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix<double, 6, 1> et;
  Eigen::Vector3d t(1., 2., 3.);
  et.block<3, 1>(0, 0) = t;
  et.block<3, 1>(3, 0) = eulerAngles;
  Eigen::Isometry3d i1 = g2o::internal::fromVectorET(et);

  Eigen::Matrix<double, 7, 1> qt2 = g2o::internal::toVectorQT(i1);

  Eigen::Isometry3d i2 = g2o::internal::fromVectorQT(qt2);
  EXPECT_NEAR(0., (i1.linear() - i2.linear()).array().abs().maxCoeff(), 1e-6);
  EXPECT_NEAR(
      0., (i1.translation() - i2.translation()).array().abs().maxCoeff(), 1e-6);
}
