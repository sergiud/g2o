// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <iterator>
#include <sstream>
#include <vector>

#include "g2o/types/sba/types_six_dof_expmap.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;
using ::testing::Le;

namespace {

/**
 * Stereo observation (u, v, u_r) followed by the upper triangle of its
 * information, each value distinct so that a shift shows.
 */
constexpr const char* Observation = "1 2 3  4 5 6 7 8 9";

template <typename Edge>
class StereoEdges : public ::testing::Test {};

using StereoEdgeTypes = ::testing::Types<g2o::EdgeStereoSE3ProjectXYZ,
                                         g2o::EdgeStereoSE3ProjectXYZOnlyPose>;
TYPED_TEST_SUITE(StereoEdges, StereoEdgeTypes);

}  // namespace

TYPED_TEST(StereoEdges, ReadsAStereoObservation) {
  TypeParam edge;
  std::istringstream is(Observation);

  ASSERT_TRUE(edge.read(is));

  EXPECT_THAT(edge.measurement(), ElementsAre(1., 2., 3.));
  const Eigen::Matrix3d& information = edge.information();
  EXPECT_THAT(information.reshaped(),
              ElementsAre(4., 5., 6., 5., 7., 8., 6., 8., 9.));
}

TYPED_TEST(StereoEdges, WritesItsThreeValues) {
  TypeParam edge;
  std::istringstream is(Observation);
  ASSERT_TRUE(edge.read(is));
  std::ostringstream os;

  ASSERT_TRUE(edge.write(os));

  std::istringstream written(os.str());
  std::vector<double> values{std::istream_iterator<double>{written},
                             std::istream_iterator<double>{}};
  EXPECT_THAT(values, ElementsAre(1., 2., 3., 4., 5., 6., 7., 8., 9.));
}

/**
 * The stereo projection keeps the precision of its double arguments, so an
 * observation it predicts exactly leaves no error.
 */
TEST(StereoEdgeOnlyPose, ProjectsInDoublePrecision) {
  constexpr double Tolerance = 1e-9;
  const Eigen::Vector3d point(0.3, 0.2, 3.7);
  g2o::VertexSE3Expmap pose;
  g2o::EdgeStereoSE3ProjectXYZOnlyPose edge;
  edge.setVertex(0, &pose);
  edge.Xw = point;
  edge.fx = 500.;
  edge.fy = 510.;
  edge.cx = 320.;
  edge.cy = 240.;
  edge.bf = 40.;
  const double u = edge.fx * point.x() / point.z() + edge.cx;
  const double v = edge.fy * point.y() / point.z() + edge.cy;
  edge.setMeasurement(Eigen::Vector3d(u, v, u - edge.bf / point.z()));

  edge.computeError();

  EXPECT_THAT(edge.error().cwiseAbs().maxCoeff(), Le(Tolerance));
}

//! The edge that also estimates the point projects in double precision too.
TEST(StereoEdge, ProjectsInDoublePrecision) {
  constexpr double Tolerance = 1e-9;
  const Eigen::Vector3d position(0.3, 0.2, 3.7);
  g2o::VertexSBAPointXYZ point;
  point.setEstimate(position);
  g2o::VertexSE3Expmap pose;
  g2o::EdgeStereoSE3ProjectXYZ edge;
  edge.setVertex(0, &point);
  edge.setVertex(1, &pose);
  edge.fx = 500.;
  edge.fy = 510.;
  edge.cx = 320.;
  edge.cy = 240.;
  edge.bf = 40.1;
  const double u = edge.fx * position.x() / position.z() + edge.cx;
  const double v = edge.fy * position.y() / position.z() + edge.cy;
  edge.setMeasurement(Eigen::Vector3d(u, v, u - edge.bf / position.z()));

  edge.computeError();

  EXPECT_THAT(edge.error().cwiseAbs().maxCoeff(), Le(Tolerance));
}
