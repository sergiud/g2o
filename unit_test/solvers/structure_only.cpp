// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/types/slam3d/edge_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "gtest/gtest.h"

/**
 * The solver holds the Hessian of a point in a matrix of the dimension it
 * was instantiated for, which a point of another dimension would overrun.
 */
TEST(StructureOnlySolver, RejectsAPointOfAnotherDimension) {
  const Eigen::Vector3d start(1., 2., 3.);
  g2o::SparseOptimizer graph;
  auto anchor = std::make_unique<g2o::VertexPointXYZ>();
  anchor->setId(0);
  anchor->setFixed(true);
  auto point = std::make_unique<g2o::VertexPointXYZ>();
  point->setId(1);
  point->setEstimate(start);
  auto* fixed = anchor.get();
  auto* free = point.get();
  ASSERT_TRUE(graph.addVertex(std::move(anchor)));
  ASSERT_TRUE(graph.addVertex(std::move(point)));
  // The solver needs a track of at least two observations.
  for (int i = 0; i < 2; ++i) {
    auto edge = std::make_unique<g2o::EdgePointXYZ>();
    edge->setVertex(0, fixed);
    edge->setVertex(1, free);
    edge->setMeasurement(Eigen::Vector3d(0.5, 0.5, 0.5));
    edge->setInformation(Eigen::Matrix3d::Identity());
    ASSERT_TRUE(graph.addEdge(std::move(edge)));
  }
  g2o::StructureOnlySolver<2> solver;
  g2o::OptimizableGraph::VertexContainer points{free};

  EXPECT_EQ(solver.calc(points, 1), g2o::OptimizationAlgorithm::Fail);
  EXPECT_EQ(free->estimate(), start);
}
