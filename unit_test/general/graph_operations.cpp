// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "gtest/gtest.h"

typedef g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType>
    SlamLinearSolver;

static g2o::SparseOptimizer* createOptimizer() {
  // Initialize the SparseOptimizer
  g2o::SparseOptimizer* mOptimizer = new g2o::SparseOptimizer();
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  mOptimizer->setAlgorithm(
      new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
  return mOptimizer;
}

TEST(General, BinaryEdgeConstructor) {
  g2o::EdgeSE3 e1;
  ASSERT_EQ(NULL, e1.vertices()[0]);
  ASSERT_EQ(NULL, e1.vertices()[1]);

  g2o::EdgeSE2 e2;
  ASSERT_EQ(NULL, e2.vertices()[0]);
  ASSERT_EQ(NULL, e2.vertices()[1]);
}

TEST(General, GraphAddVertex) {
  g2o::SparseOptimizer* optimizer = createOptimizer();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_EQ(optimizer, v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_FALSE(optimizer->addVertex(v1));
  ASSERT_EQ(size_t(1), optimizer->vertices().size());

  {
    g2o::VertexSE2* v2 = new g2o::VertexSE2();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(NULL, v2->graph());
    delete v2;
  }

  delete optimizer;
}

TEST(General, GraphAddEdge) {
  g2o::SparseOptimizer* optimizer = createOptimizer();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  g2o::VertexSE2* v2 = new g2o::VertexSE2();
  v2->setId(1);

  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_TRUE(optimizer->addVertex(v2));

  g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  ASSERT_TRUE(optimizer->addEdge(e1));
  ASSERT_EQ(optimizer, e1->graph());
  ASSERT_FALSE(optimizer->addEdge(e1));
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().count(e1));
  ASSERT_EQ(size_t(1), v2->edges().count(e1));

  g2o::EdgeSE2* e2 = new g2o::EdgeSE2();
  ASSERT_FALSE(optimizer->addEdge(e2))
      << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(NULL, e2->graph());

  delete optimizer;
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
    auto* vertex = new CountedVertex(destructions);
    vertex->setId(id);
    graph.addVertex(vertex);
  }
  auto* edge = new CountedEdge(destructions);
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));
  graph.addEdge(edge);
  return edge;
}

}  // namespace

/**
 * A graph owns the vertices and edges added to it, so destroying the graph
 * frees them.
 */
TEST(General, GraphDeletesItsElementsOnDestruction) {
  Destructions destructions;
  {
    g2o::SparseOptimizer graph;
    addTwoVerticesAndEdge(graph, destructions);
  }

  EXPECT_EQ(destructions.vertices, 2);
  EXPECT_EQ(destructions.edges, 1);
}

//! Clearing a graph frees its vertices and edges.
TEST(General, GraphClearDeletesItsElements) {
  Destructions destructions;
  g2o::SparseOptimizer graph;
  addTwoVerticesAndEdge(graph, destructions);

  graph.clear();

  EXPECT_EQ(destructions.vertices, 2);
  EXPECT_EQ(destructions.edges, 1);
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

/**
 * An edge whose parameters the graph lacks is rejected, so the caller keeps
 * owning it and the graph keeps no reference to it.
 */
TEST(General, GraphAddEdgeWithoutItsParameterLeavesTheEdgeToTheCaller) {
  g2o::SparseOptimizer graph;
  auto* pose = new g2o::VertexSE3;
  pose->setId(0);
  ASSERT_TRUE(graph.addVertex(pose));
  auto* point = new g2o::VertexPointXYZ;
  point->setId(1);
  ASSERT_TRUE(graph.addVertex(point));
  auto edge = std::make_unique<g2o::EdgeSE3PointXYZ>();
  edge->setVertex(0, pose);
  edge->setVertex(1, point);
  constexpr int missingParameterId = 0;
  ASSERT_TRUE(edge->setParameterId(0, missingParameterId));

  EXPECT_FALSE(graph.addEdge(edge.get()));

  EXPECT_TRUE(graph.edges().empty());
  EXPECT_TRUE(pose->edges().empty());
  EXPECT_TRUE(point->edges().empty());
}
