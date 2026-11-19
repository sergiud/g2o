// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/icp/types_icp.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

constexpr double Tolerance = 1e-12;

//! Transform from the world into the frame of a camera at the given pose.
Eigen::Matrix<double, 3, 4> worldToCamera(const Eigen::Isometry3d& pose) {
  return pose.inverse().matrix().topRows<3>();
}

const Eigen::Isometry3d Pose = Eigen::Translation3d(1., 2., 3.) *
                               Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY());

}  // namespace

/**
 * Levenberg-Marquardt restores the estimate of a rejected step, and the
 * projection of the camera must follow it.
 */
TEST(VertexSCam, RestoresItsTransformsWithItsEstimate) {
  g2o::VertexSCam camera;
  camera.setEstimate(Pose);
  camera.push();
  const g2o::Vector6d step =
      (g2o::Vector6d() << 1., 0., 0., 0., 0., 0.).finished();
  camera.oplus(step.data());

  camera.pop();

  EXPECT_THAT(camera.w2n,
              DistanceFrom(worldToCamera(Pose), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

TEST(VertexSCam, SetsItsTransformsWithItsEstimate) {
  g2o::VertexSCam camera;

  camera.setEstimate(Pose);

  EXPECT_THAT(camera.w2n,
              DistanceFrom(worldToCamera(Pose), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}
