// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/factory.h"

#include <memory>
#include <sstream>
#include <string>
#include <utility>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

/**
 * The listing of the registered types is output for the caller, so every
 * type name goes to the stream the caller passes.
 */
TEST(General, FactoryPrintsRegisteredTypesToTheGivenStream) {
  // A vertex of the library makes sure that its types are registered.
  g2o::VertexSE2 vertex;
  std::ostringstream listing;

  g2o::Factory::instance()->printRegisteredTypes(listing, true);

  EXPECT_THAT(listing.str(), testing::HasSubstr("#\tVERTEX_SE2\n"));
}

/**
 * Graph files separate their tokens by whitespace, so a type tag that
 * contains whitespace cannot be read back.
 */
TEST(General, FactoryTagsContainNoWhitespace) {
  // Vertices of the type libraries make sure that their types are registered.
  g2o::VertexSE3 vertexSE3;
  g2o::VertexSE3Expmap vertexSE3Expmap;
  g2o::VertexSim3Expmap vertexSim3Expmap;
  std::ostringstream listing;

  g2o::Factory::instance()->printRegisteredTypes(listing, false);

  std::istringstream lines(listing.str());
  std::string line;
  std::getline(lines, line);  // The heading of the listing.
  while (std::getline(lines, line)) {
    const std::string tag = line.substr(1);
    EXPECT_EQ(tag.find_first_of(" \t"), std::string::npos)
        << "tag \"" << tag << "\" contains whitespace";
  }
}

/**
 * A graph of 3D poses written by save() loads into a graph with the same
 * vertices and edges.
 */
TEST(General, GraphOfSE3PosesSurvivesSaveAndLoad) {
  g2o::SparseOptimizer graph;
  for (int id = 0; id < 2; ++id) {
    auto vertex = std::make_unique<g2o::VertexSE3>();
    vertex->setId(id);
    ASSERT_TRUE(graph.addVertex(std::move(vertex)));
  }
  auto edge = std::make_unique<g2o::EdgeSE3>();
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));
  ASSERT_TRUE(graph.addEdge(std::move(edge)));
  std::stringstream file;
  ASSERT_TRUE(graph.save(file));

  g2o::SparseOptimizer loaded;
  ASSERT_TRUE(loaded.load(file));

  EXPECT_EQ(loaded.vertices().size(), 2U);
  EXPECT_EQ(loaded.edges().size(), 1U);
}
