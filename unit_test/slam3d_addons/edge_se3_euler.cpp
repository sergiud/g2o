// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/slam3d_addons/edge_se3_euler.h"

#include <cmath>
#include <sstream>
#include <string>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::DoubleNear;
using ::testing::Le;

namespace {

//! Pure rotation about the z axis by 3 rad, close to a half turn.
constexpr const char* YawMeasurement = "0 0 0 0 0 3";
//! Unit information on the translation and the Euler angles.
constexpr const char* UnitInformation =
    " 1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1";
constexpr double Yaw = 3.;
constexpr double Tolerance = 1e-6;

}  // namespace

/**
 * The edge converts the information on the Euler angles into the minimal
 * quaternion parameters of the pose, whose last component is sin(yaw/2) for
 * a rotation about z, so the derivative of the yaw is 2/cos(yaw/2).
 */
TEST(EdgeSE3Euler, ConvertsTheInformationOfTheYaw) {
  g2o::EdgeSE3Euler edge;
  std::istringstream is(std::string{YawMeasurement} + UnitInformation);
  ASSERT_TRUE(edge.read(is));

  const double derivative = 2. / std::cos(Yaw / 2.);
  EXPECT_THAT(
      edge.information()(5, 5),
      DoubleNear(derivative * derivative, Tolerance * derivative * derivative));
}

//! Writing converts the information back into that of the Euler angles.
TEST(EdgeSE3Euler, WritesTheInformationItRead) {
  g2o::EdgeSE3Euler edge;
  std::istringstream is(std::string{YawMeasurement} + UnitInformation);
  ASSERT_TRUE(edge.read(is));
  std::ostringstream os;
  ASSERT_TRUE(edge.write(os));

  std::istringstream written(os.str());
  Eigen::Matrix<double, 6, 1> measurement;
  for (int i = 0; i < 6; ++i) {
    written >> measurement[i];
  }
  Eigen::Matrix<double, 6, 6> information;
  for (int i = 0; i < 6; ++i) {
    for (int j = i; j < 6; ++j) {
      written >> information(i, j);
      information(j, i) = information(i, j);
    }
  }
  ASSERT_FALSE(written.fail());
  EXPECT_THAT(information,
              DistanceFrom(Eigen::Matrix<double, 6, 6>::Identity(),
                           g2o::test::MaxAbsDifference, Le(Tolerance)));
}
