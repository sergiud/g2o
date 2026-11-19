// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include <g2o_incremental/graph_optimizer_sparse_incremental.h>
#include <g2o_interactive/g2o_slam_interface.h>

#include <vector>

#include "gtest/gtest.h"

namespace {

//! Dimension of a planar pose.
constexpr int PlanarDimension = 3;

//! Adds a planar edge with an identity measurement and unit information.
bool addPlanarEdge(g2o::G2oSlamInterface& slam, int from, int to) {
  // The upper triangle of the information row by row.
  const std::vector<double> information{1., 0., 0., 1., 0., 1.};
  return slam.addEdge("EDGE_XYT", 0, PlanarDimension, from, to, {0., 0., 0.},
                      information);
}

}  // namespace

/**
 * An update whose edges touch only fixed vertices leaves the factorization as
 * it is, since it has no block of a free vertex to extend.
 */
TEST(IncrementalSlam, UpdatesEdgesBetweenFixedVertices) {
  g2o::SparseOptimizerIncremental optimizer;
  g2o::G2oSlamInterface slam(&optimizer);
  slam.setUpdateGraphEachN(1);

  ASSERT_TRUE(slam.addNode("VERTEX_XYT", 0, PlanarDimension, {}));
  ASSERT_TRUE(addPlanarEdge(slam, 0, 1));
  ASSERT_TRUE(slam.solveState());
  ASSERT_TRUE(slam.fixNode({1}));
  ASSERT_TRUE(addPlanarEdge(slam, 1, 2));
  ASSERT_TRUE(slam.fixNode({2}));

  EXPECT_TRUE(slam.solveState());
}
