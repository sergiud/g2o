// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/types/sba/types_sba.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

constexpr double Tolerance = 1e-12;

//! Translation (1, 2, 3) without rotation, which the camera undoes.
const Eigen::Vector3d Translation(1., 2., 3.);

}  // namespace

/**
 * Every way of setting the pose of a camera updates the transform from the
 * world into the camera, which the projections use.
 */
TEST(VertexCam, SetsItsTransformWithItsEstimateData) {
  g2o::VertexCam camera;
  const g2o::Vector7d data =
      (g2o::Vector7d() << Translation, 0., 0., 0., 1.).finished();

  ASSERT_TRUE(camera.setEstimateData(data.data()));

  EXPECT_THAT(Eigen::Vector3d(camera.estimate().w2n.col(3)),
              DistanceFrom(Eigen::Vector3d(-Translation),
                           g2o::test::MaxAbsDifference, Le(Tolerance)));
}

TEST(VertexCam, SetsItsTransformWithItsMinimalEstimateData) {
  g2o::VertexCam camera;
  const g2o::Vector6d data =
      (g2o::Vector6d() << Translation, 0., 0., 0.).finished();

  ASSERT_TRUE(camera.setMinimalEstimateData(data.data()));

  EXPECT_THAT(Eigen::Vector3d(camera.estimate().w2n.col(3)),
              DistanceFrom(Eigen::Vector3d(-Translation),
                           g2o::test::MaxAbsDifference, Le(Tolerance)));
}
