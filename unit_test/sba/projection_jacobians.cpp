// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <limits>
#include <memory>
#include <type_traits>
#include <utility>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_sba.h"
#include "gtest/gtest.h"

namespace {

template <typename Edge>
class ProjectionJacobian : public ::testing::Test {};

using ProjectionEdges =
    ::testing::Types<g2o::EdgeProjectP2MC, g2o::EdgeProjectP2SC,
                     g2o::EdgeProjectP2MC_Intrinsics>;
TYPED_TEST_SUITE(ProjectionJacobian, ProjectionEdges);

}  // namespace

/**
 * A point without a valid position has no Jacobian, which the solver
 * rejects, while the program that hosts the optimization keeps running.
 */
TYPED_TEST(ProjectionJacobian, LinearizesAnInvalidPointWithoutAborting) {
  constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
  g2o::SparseOptimizer graph;
  auto point = std::make_unique<g2o::VertexSBAPointXYZ>();
  point->setId(0);
  point->setEstimate(Eigen::Vector3d(0., 0., NaN));
  auto camera = std::make_unique<g2o::VertexCam>();
  camera->setId(1);
  camera->setEstimate(g2o::SBACam());
  auto edge = std::make_unique<TypeParam>();
  edge->setVertex(0, point.get());
  edge->setVertex(1, camera.get());
  if constexpr (std::is_same_v<TypeParam, g2o::EdgeProjectP2MC_Intrinsics>) {
    auto intrinsics = std::make_unique<g2o::VertexIntrinsics>();
    intrinsics->setId(2);
    intrinsics->setToOrigin();
    edge->setVertex(2, intrinsics.get());
    ASSERT_TRUE(graph.addVertex(std::move(intrinsics)));
  }
  TypeParam* added = edge.get();
  ASSERT_TRUE(graph.addVertex(std::move(point)));
  ASSERT_TRUE(graph.addVertex(std::move(camera)));
  ASSERT_TRUE(graph.addEdge(std::move(edge)));
  g2o::JacobianWorkspace workspace;
  workspace.updateSize(added);
  workspace.allocate();

  static_cast<g2o::OptimizableGraph::Edge*>(added)->linearizeOplus(workspace);

  SUCCEED();
}
