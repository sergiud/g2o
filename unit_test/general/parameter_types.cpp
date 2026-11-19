// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <utility>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gtest/gtest.h"

namespace {

//! A pose observes a point through the parameter of the graph with id 0.
class EdgeParameter : public ::testing::Test {
 protected:
  //! Adds the parameter and an edge that uses it, and returns whether the
  //! graph accepted the edge.
  bool addEdgeWith(std::unique_ptr<g2o::Parameter> parameter) {
    parameter->setId(0);
    EXPECT_TRUE(_graph.addParameter(std::move(parameter)));
    auto pose = std::make_unique<g2o::VertexSE3>();
    pose->setId(0);
    auto point = std::make_unique<g2o::VertexPointXYZ>();
    point->setId(1);
    auto edge = std::make_unique<g2o::EdgeSE3PointXYZ>();
    edge->setVertex(0, pose.get());
    edge->setVertex(1, point.get());
    edge->setParameterId(0, 0);
    _edge = edge.get();
    EXPECT_TRUE(_graph.addVertex(std::move(pose)));
    EXPECT_TRUE(_graph.addVertex(std::move(point)));
    return _graph.addEdge(std::move(edge));
  }

  g2o::SparseOptimizer _graph;
  g2o::EdgeSE3PointXYZ* _edge = nullptr;
};

}  // namespace

//! The edge refuses a parameter of a type it cannot use.
TEST_F(EdgeParameter, RejectsAParameterOfAnotherType) {
  EXPECT_FALSE(addEdgeWith(std::make_unique<g2o::ParameterSE2Offset>()));
}

//! A parameter derived from the expected type serves the edge as well.
TEST_F(EdgeParameter, AcceptsAParameterOfADerivedType) {
  auto camera = std::make_unique<g2o::ParameterCamera>();
  g2o::ParameterCamera* expected = camera.get();

  ASSERT_TRUE(addEdgeWith(std::move(camera)));

  EXPECT_EQ(_edge->parameter(0), expected);
}

/**
 * An edge that uses its parameter only while computing its error refuses a
 * parameter of another type when added, rather than when evaluated.
 */
TEST(EdgeParameterOfAProjection, RejectsAParameterOfAnotherType) {
  g2o::SparseOptimizer graph;
  auto parameter = std::make_unique<g2o::ParameterSE3Offset>();
  parameter->setId(0);
  ASSERT_TRUE(graph.addParameter(std::move(parameter)));
  auto point = std::make_unique<g2o::VertexSBAPointXYZ>();
  point->setId(0);
  auto pose = std::make_unique<g2o::VertexSE3Expmap>();
  pose->setId(1);
  auto edge = std::make_unique<g2o::EdgeProjectXYZ2UV>();
  edge->setVertex(0, point.get());
  edge->setVertex(1, pose.get());
  edge->setParameterId(0, 0);
  ASSERT_TRUE(graph.addVertex(std::move(point)));
  ASSERT_TRUE(graph.addVertex(std::move(pose)));

  EXPECT_FALSE(graph.addEdge(std::move(edge)));
}
