// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <sstream>
#include <string>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/data/vertex_tag.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::IsNull;
using ::testing::SizeIs;

namespace {

//! Two planar poses and the odometry between them.
constexpr const char* Chain =
    "VERTEX_SE2 0 0 0 0\n"
    "VERTEX_SE2 1 1 0 0\n"
    "EDGE_SE2 0 1 1 0 0 1 0 0 1 0 1\n";

}  // namespace

/**
 * A line of blanks or a carriage return of a file with Windows line endings
 * holds no element, so it repeats none of the line before.
 */
TEST(GraphLoad, IgnoresLinesOfWhitespace) {
  for (const std::string blank : {"   \n", "\r\n", "\t\n"}) {
    SCOPED_TRACE(::testing::PrintToString(blank));
    g2o::SparseOptimizer graph;
    std::istringstream file(std::string{Chain} + blank);

    EXPECT_TRUE(graph.load(file));

    EXPECT_THAT(graph.vertices(), SizeIs(2));
    EXPECT_THAT(graph.edges(), SizeIs(1));
  }
}

/**
 * An edge with any number of vertices lists their ids up to a "||", and a
 * line that lacks it ends the list at its end instead of reading on forever.
 */
TEST(GraphLoad, RejectsAVertexListWithoutItsEnd) {
  g2o::SparseOptimizer graph;
  std::istringstream file(std::string{Chain} + "EDGE_SE2_LOTSOFXY 0 1\n");

  graph.load(file);

  EXPECT_THAT(graph.edges(), SizeIs(1));
}

class GraphLoadFailure : public ::testing::TestWithParam<std::string> {};

/**
 * A line that the graph cannot read or add loses data, which the caller
 * learns from the result, while the remaining lines still load.
 */
TEST_P(GraphLoadFailure, ReportsALineItCannotLoad) {
  g2o::SparseOptimizer graph;
  std::istringstream file(std::string{Chain} + GetParam());

  EXPECT_FALSE(graph.load(file));

  EXPECT_THAT(graph.vertices(), SizeIs(2));
  EXPECT_THAT(graph.edges(), SizeIs(1));
}

INSTANTIATE_TEST_SUITE_P(
    GraphLoad, GraphLoadFailure,
    ::testing::Values("NO_SUCH_TYPE 1 2 3\n", "VERTEX_SE2 1 5 5 0\n",
                      "EDGE_SE2_LOTSOFXY 0 1\n", "VERTEX_SE2 -1 0 0 0\n",
                      "EDGE_SE2_LOTSOFXY 0 7 || 1 0 1 0 1 0 0 0 0 1 0 0 0 0 0 "
                      "0 1 0 0 0 1 0 0 "
                      "1 0 1\n",
                      "FIX 9\n",
                      "PARAMS_SE2OFFSET 0 0 0 0\nPARAMS_SE2OFFSET 0 0 0 0\n",
                      "EDGE_SE2 0 -1 1 0 0 1 0 0 1 0 1\n", "VERTEX_SE2 2 1 2\n",
                      "PARAMS_SE2OFFSET 0 1 2\n",
                      "EDGE_SE2_LOTSOFXY 0 1 || 1 0 1\n",
                      "EDGE_SE2 0 1 1 0 0 1 0 0 1\n", "VERTEX_TAG t 0 0\n"));

//! Without creating vertices, an edge needs the vertices it connects.
TEST(GraphLoad, ReportsAnEdgeOfAMissingVertex) {
  g2o::SparseOptimizer graph;
  std::istringstream file(std::string{Chain} +
                          "EDGE_SE2 0 5 1 0 0 1 0 0 1 0 1\n");

  EXPECT_FALSE(graph.load(file, /*createEdges=*/false));

  EXPECT_THAT(graph.vertices(), SizeIs(2));
  EXPECT_THAT(graph.edges(), SizeIs(1));
}

//! Data such as a tag belongs to the element of the line before.
TEST(GraphLoad, ReportsDataWithoutAnElement) {
  g2o::SparseOptimizer graph;
  std::istringstream file("VERTEX_TAG t 0 0 0 0 0 0 0 h 0\n" +
                          std::string{Chain});

  EXPECT_FALSE(graph.load(file));

  EXPECT_THAT(graph.vertices(), SizeIs(2));
}

/**
 * Data that follows an element the graph rejected belongs to that element,
 * not to the element loaded before it.
 */
TEST(GraphLoad, DropsTheDataOfARejectedElement) {
  g2o::SparseOptimizer graph;
  std::istringstream file(std::string{Chain} +
                          "VERTEX_SE2 1 5 5 0\n"
                          "VERTEX_TAG t 0 0 0 0 0 0 0 h 0\n");

  EXPECT_FALSE(graph.load(file));

  ASSERT_THAT(graph.edges(), SizeIs(1));
  EXPECT_THAT(static_cast<const g2o::OptimizableGraph::Edge*>(
                  graph.edges().begin()->get())
                  ->userData(),
              IsNull());
  EXPECT_THAT(graph.vertex(1)->userData(), IsNull());
}
