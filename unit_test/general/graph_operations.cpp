// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gtest/gtest.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>
    SlamLinearSolver;

static std::unique_ptr<g2o::SparseOptimizer> createOptimizer() {
  // Initialize the SparseOptimizer
  auto mOptimizer = std::make_unique<g2o::SparseOptimizer>();
  auto linearSolver = std::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
  mOptimizer->setAlgorithm(
      std::make_unique<g2o::OptimizationAlgorithmGaussNewton>(
          std::move(blockSolver)));
  return mOptimizer;
}

namespace {

//! Counts how often the elements of a graph are destroyed.
struct Destructions {
  int vertices = 0;
  int edges = 0;
};

//! Vertex that counts its destruction.
class CountedVertex : public g2o::VertexSE2 {
 public:
  explicit CountedVertex(Destructions& destructions)
      : _destructions(destructions) {}
  ~CountedVertex() override { ++_destructions.vertices; }

 private:
  Destructions& _destructions;
};

//! Edge that counts its destruction.
class CountedEdge : public g2o::EdgeSE2 {
 public:
  explicit CountedEdge(Destructions& destructions)
      : _destructions(destructions) {}
  ~CountedEdge() override { ++_destructions.edges; }

 private:
  Destructions& _destructions;
};

//! Adds two vertices joined by an edge to the graph and returns the edge.
CountedEdge* addTwoVerticesAndEdge(g2o::SparseOptimizer& graph,
                                   Destructions& destructions) {
  for (int id = 0; id < 2; ++id) {
    auto vertex = std::make_unique<CountedVertex>(destructions);
    vertex->setId(id);
    EXPECT_TRUE(graph.addVertex(std::move(vertex)));
  }
  auto edgeOwner = std::make_unique<CountedEdge>(destructions);
  auto* edge = edgeOwner.get();
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));
  EXPECT_TRUE(graph.addEdge(std::move(edgeOwner)));
  return edge;
}

}  // namespace

TEST(General, BinaryEdgeConstructor) {
  g2o::EdgeSE3 e1;
  ASSERT_EQ(NULL, e1.vertices()[0]);
  ASSERT_EQ(NULL, e1.vertices()[1]);

  g2o::EdgeSE2 e2;
  ASSERT_EQ(NULL, e2.vertices()[0]);
  ASSERT_EQ(NULL, e2.vertices()[1]);
}

TEST(General, GraphAddVertex) {
  Destructions destructions;
  std::unique_ptr<g2o::SparseOptimizer> optimizer = createOptimizer();

  auto v1Owner = std::make_unique<CountedVertex>(destructions);
  CountedVertex* v1 = v1Owner.get();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(std::move(v1Owner)));
  ASSERT_EQ(optimizer.get(), v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  // Adding a vertex the graph owns already fails and keeps the vertex.
  ASSERT_FALSE(optimizer->addVertex(std::unique_ptr<CountedVertex>(v1)));
  EXPECT_EQ(destructions.vertices, 0);
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_EQ(optimizer.get(), v1->graph());

  {
    auto v2 = std::make_unique<g2o::VertexSE2>();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(std::move(v2)));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
  }
}

TEST(General, GraphAddEdge) {
  Destructions destructions;
  std::unique_ptr<g2o::SparseOptimizer> optimizer = createOptimizer();

  auto v1Owner = std::make_unique<g2o::VertexSE2>();
  g2o::VertexSE2* v1 = v1Owner.get();
  v1->setId(0);
  auto v2Owner = std::make_unique<g2o::VertexSE2>();
  g2o::VertexSE2* v2 = v2Owner.get();
  v2->setId(1);

  ASSERT_TRUE(optimizer->addVertex(std::move(v1Owner)));
  ASSERT_TRUE(optimizer->addVertex(std::move(v2Owner)));

  auto e1Owner = std::make_unique<CountedEdge>(destructions);
  CountedEdge* e1 = e1Owner.get();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  ASSERT_TRUE(optimizer->addEdge(std::move(e1Owner)));
  ASSERT_EQ(optimizer.get(), e1->graph());
  // Adding an edge the graph owns already fails and keeps the edge.
  ASSERT_FALSE(optimizer->addEdge(std::unique_ptr<CountedEdge>(e1)));
  EXPECT_EQ(destructions.edges, 0);
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v2->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().count(e1));
  ASSERT_EQ(size_t(1), v2->edges().count(e1));

  auto e2 = std::make_unique<g2o::EdgeSE2>();
  ASSERT_FALSE(optimizer->addEdge(std::move(e2)))
      << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
}

/**
 * A graph owns the vertices and edges added to it, so destroying the graph
 * frees them.
 */
TEST(General, GraphDeletesItsElementsOnDestruction) {
  Destructions destructions;
  {
    g2o::SparseOptimizer graph;
    addTwoVerticesAndEdge(graph, destructions);
    EXPECT_EQ(destructions.vertices, 0);
    EXPECT_EQ(destructions.edges, 0);
  }

  EXPECT_EQ(destructions.vertices, 2);
  EXPECT_EQ(destructions.edges, 1);
}

//! Clearing a graph frees its vertices and edges.
TEST(General, GraphClearDeletesItsElements) {
  Destructions destructions;
  g2o::SparseOptimizer graph;
  addTwoVerticesAndEdge(graph, destructions);
  ASSERT_EQ(destructions.vertices, 0);
  ASSERT_EQ(destructions.edges, 0);

  graph.clear();

  EXPECT_EQ(destructions.vertices, 2);
  EXPECT_EQ(destructions.edges, 1);
  EXPECT_TRUE(graph.vertices().empty());
  EXPECT_TRUE(graph.edges().empty());
}

//! Removing an edge frees it and detaches it from the vertices it keeps.
TEST(General, GraphRemoveEdgeDeletesTheEdge) {
  Destructions destructions;
  g2o::SparseOptimizer graph;
  CountedEdge* edge = addTwoVerticesAndEdge(graph, destructions);

  ASSERT_TRUE(graph.removeEdge(edge));

  EXPECT_EQ(destructions.vertices, 0);
  EXPECT_EQ(destructions.edges, 1);
  EXPECT_TRUE(graph.edges().empty());
  EXPECT_TRUE(graph.vertex(0)->edges().empty());
  EXPECT_TRUE(graph.vertex(1)->edges().empty());
}

//! Removing a vertex frees it together with the edges it belongs to.
TEST(General, GraphRemoveVertexDeletesItsEdges) {
  Destructions destructions;
  g2o::SparseOptimizer graph;
  addTwoVerticesAndEdge(graph, destructions);

  ASSERT_TRUE(graph.removeVertex(graph.vertex(0)));

  EXPECT_EQ(destructions.vertices, 1);
  EXPECT_EQ(destructions.edges, 1);
}

//! Moving a vertex to an id another vertex has fails and keeps both vertices.
TEST(General, GraphChangeIdToATakenIdKeepsBothVertices) {
  Destructions destructions;
  g2o::SparseOptimizer graph;
  addTwoVerticesAndEdge(graph, destructions);
  g2o::HyperGraph::Vertex* vertex = graph.vertex(0);

  EXPECT_FALSE(graph.changeId(vertex, 1));

  EXPECT_EQ(destructions.vertices, 0);
  EXPECT_EQ(graph.vertex(0), vertex);
  EXPECT_NE(graph.vertex(1), nullptr);
  EXPECT_EQ(graph.vertices().size(), 2U);
}
