// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_sba.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"
#include "unit_test/test_helper/evaluate_jacobian.h"

using ::testing::DistanceFrom;
using ::testing::Le;

namespace {

//! A camera observes a point through intrinsics it estimates.
class IntrinsicsEdge : public ::testing::Test {
 protected:
  void SetUp() override {
    auto point = std::make_unique<g2o::VertexSBAPointXYZ>();
    point->setId(0);
    point->setEstimate(Eigen::Vector3d(0.4, -0.3, 5.));
    _point = point.get();
    auto camera = std::make_unique<g2o::VertexCam>();
    camera->setId(1);
    g2o::SBACam pose(Eigen::Quaterniond(Eigen::AngleAxisd(
                         0.1, Eigen::Vector3d(1., 2., 3.).normalized())),
                     Eigen::Vector3d(0.2, 0.1, -0.5));
    camera->setEstimate(pose);
    _camera = camera.get();
    auto intrinsics = std::make_unique<g2o::VertexIntrinsics>();
    intrinsics->setId(2);
    Eigen::Matrix<double, 5, 1> values;
    values << 500., 450., 320., 240., 0.1;
    intrinsics->setEstimate(values);
    _intrinsics = intrinsics.get();
    auto edge = std::make_unique<g2o::EdgeProjectP2MC_Intrinsics>();
    edge->setVertex(0, _point);
    edge->setVertex(1, _camera);
    edge->setVertex(2, _intrinsics);
    _edge = edge.get();
    ASSERT_TRUE(_graph.addVertex(std::move(point)));
    ASSERT_TRUE(_graph.addVertex(std::move(camera)));
    ASSERT_TRUE(_graph.addVertex(std::move(intrinsics)));
    ASSERT_TRUE(_graph.addEdge(std::move(edge)));
  }

  //! Projection of the point with the estimated intrinsics.
  Eigen::Vector2d projection() const {
    const Eigen::Vector3d p =
        _camera->estimate().w2n * _point->estimate().homogeneous();
    const auto& k = _intrinsics->estimate();
    return {k[0] * p.x() / p.z() + k[2], k[1] * p.y() / p.z() + k[3]};
  }

  g2o::SparseOptimizer _graph;
  g2o::VertexSBAPointXYZ* _point = nullptr;
  g2o::VertexCam* _camera = nullptr;
  g2o::VertexIntrinsics* _intrinsics = nullptr;
  g2o::EdgeProjectP2MC_Intrinsics* _edge = nullptr;
};

}  // namespace

/**
 * The edge projects with the intrinsics it estimates rather than those the
 * camera holds, which would leave the intrinsics without effect.
 */
TEST_F(IntrinsicsEdge, ProjectsWithTheEstimatedIntrinsics) {
  constexpr double Tolerance = 1e-9;
  _edge->setMeasurement(projection());

  _edge->computeError();

  EXPECT_THAT(_edge->error(),
              DistanceFrom(Eigen::Vector2d::Zero(), g2o::test::MaxAbsDifference,
                           Le(Tolerance)));
}

TEST_F(IntrinsicsEdge, Jacobian) {
  // The image coordinates reach hundreds of pixels, which scales the error
  // of the numeric differentiation.
  constexpr double Tolerance = 1e-4;
  _edge->setMeasurement(projection());
  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(_edge);
  numericJacobianWorkspace.allocate();

  g2o::evaluateJacobianMulti(*_edge, jacobianWorkspace,
                             numericJacobianWorkspace, Tolerance);
}
