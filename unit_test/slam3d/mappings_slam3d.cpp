// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <span>

#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::DoubleEq;
using ::testing::Eq;
using ::testing::Le;
using ::testing::Pointwise;

using namespace std;

TEST(MappingsSlam3D, EulerConversion) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);
  Eigen::Vector3d eulerAnglesFromMatrix = g2o::internal::toEuler(m1);
  EXPECT_THAT(eulerAnglesFromMatrix,
              Pointwise(DoubleEq(), std::span(eulerAngles.data(), 3)));
}

TEST(MappingsSlam3D, QuaternionConversion) {
  // A yaw near pi gives a quaternion with a negative real part, which the
  // compact form normalizes.
  Eigen::Vector3d eulerAngles(.1, .2, -3.);
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);

  Eigen::Vector3d q = g2o::internal::toCompactQuaternion(m1);
  Eigen::Matrix3d m2 = g2o::internal::fromCompactQuaternion(q);
  EXPECT_THAT(m2, DistanceFrom(m1, g2o::test::MaxAbsDifference, Le(1e-12)));
}

TEST(MappingsSlam3D, ET) {
  Eigen::Vector3d eulerAngles(.1, .2, .3);
  Eigen::Matrix<double, 6, 1> et;
  Eigen::Vector3d t(1., 2., 3.);
  et.block<3, 1>(0, 0) = t;
  et.block<3, 1>(3, 0) = eulerAngles;
  Eigen::Matrix3d m1 = g2o::internal::fromEuler(eulerAngles);

  Eigen::Isometry3d i1 = g2o::internal::fromVectorET(et);
  EXPECT_THAT(Eigen::Vector3d(i1.translation()),
              DistanceFrom(t, g2o::test::MaxAbsDifference, Eq(0.)));
  EXPECT_THAT(Eigen::Matrix3d(i1.linear()),
              DistanceFrom(m1, g2o::test::MaxAbsDifference, Le(1e-6)));

  Eigen::Matrix<double, 6, 1> et2 = g2o::internal::toVectorET(i1);
  EXPECT_THAT(et2, DistanceFrom(et, g2o::test::MaxAbsDifference, Le(1e-6)));
}

TEST(MappingsSlam3D, MQT) {
  // A yaw near pi gives a quaternion with a negative real part.
  Eigen::Vector3d eulerAngles(.1, .2, -3.);
  Eigen::Matrix<double, 6, 1> et;
  Eigen::Vector3d t(1., 2., 3.);
  et.block<3, 1>(0, 0) = t;
  et.block<3, 1>(3, 0) = eulerAngles;
  Eigen::Isometry3d i1 = g2o::internal::fromVectorET(et);

  Eigen::Matrix<double, 6, 1> qt1 = g2o::internal::toVectorMQT(i1);

  Eigen::Isometry3d i2 = g2o::internal::fromVectorMQT(qt1);
  EXPECT_THAT(i2.matrix(),
              DistanceFrom(i1.matrix(), g2o::test::MaxAbsDifference, Le(1e-6)));
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
  EXPECT_THAT(i2.matrix(),
              DistanceFrom(i1.matrix(), g2o::test::MaxAbsDifference, Le(1e-6)));
}
