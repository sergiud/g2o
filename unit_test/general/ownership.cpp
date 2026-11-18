// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include "g2o/core/block_solver.h"
#include "g2o/core/creators.h"
#include "g2o/core/factory.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/core/optimization_algorithm_dogleg.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/parameter.h"
#include "g2o/core/robust_kernel_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/stuff/property.h"
#include "g2o/types/data/data_queue.h"
#include "g2o/types/data/robot_data.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gtest/gtest.h"

namespace {

using BlockSolver = g2o::BlockSolverX;
using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

//! Gauss-Newton algorithm that counts its destruction.
class CountedAlgorithm : public g2o::OptimizationAlgorithmGaussNewton {
 public:
  CountedAlgorithm(std::unique_ptr<g2o::Solver> solver, int& destructions)
      : OptimizationAlgorithmGaussNewton(std::move(solver)),
        _destructions(destructions) {}
  ~CountedAlgorithm() override { ++_destructions; }

 private:
  int& _destructions;
};

std::unique_ptr<g2o::Solver> createSolver() {
  return std::make_unique<BlockSolver>(std::make_unique<LinearSolver>());
}

//! User data that counts its destruction.
class CountedData : public g2o::HyperGraph::Data {
 public:
  explicit CountedData(int& destructions) : _destructions(destructions) {}
  ~CountedData() override { ++_destructions; }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }

 private:
  int& _destructions;
};

//! Parameter that counts its destruction.
class CountedParameter : public g2o::Parameter {
 public:
  CountedParameter(int id, int& destructions) : _destructions(destructions) {
    setId(id);
  }
  ~CountedParameter() override { ++_destructions; }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }

 private:
  int& _destructions;
};

//! 3D pose that counts its destruction.
class CountedVertex : public g2o::VertexSE3 {
 public:
  CountedVertex(int id, int& destructions) : _destructions(destructions) {
    setId(id);
  }
  ~CountedVertex() override { ++_destructions; }

 private:
  int& _destructions;
};

//! Edge to a 3D point that counts its destruction.
class CountedEdge : public g2o::EdgeSE3PointXYZ {
 public:
  explicit CountedEdge(int& destructions) : _destructions(destructions) {}
  ~CountedEdge() override { ++_destructions; }

 private:
  int& _destructions;
};

//! Property that counts its destruction.
class CountedProperty : public g2o::IntProperty {
 public:
  CountedProperty(const std::string& name, int& destructions)
      : g2o::IntProperty(name, 0), _destructions(destructions) {}
  ~CountedProperty() override { ++_destructions; }

 private:
  int& _destructions;
};

//! Robot data that counts its destruction.
class CountedRobotData : public g2o::RobotData {
 public:
  CountedRobotData(double timestamp, int& destructions)
      : _destructions(destructions) {
    setTimestamp(timestamp);
  }
  ~CountedRobotData() override { ++_destructions; }
  bool read(std::istream&) override { return true; }
  bool write(std::ostream&) const override { return true; }

 private:
  int& _destructions;
};

//! Edge whose caches never resolve and that counts its destruction.
class EdgeWithoutCaches : public g2o::EdgeSE2 {
 public:
  explicit EdgeWithoutCaches(int& destructions) : _destructions(destructions) {}
  ~EdgeWithoutCaches() override { ++_destructions; }
  bool resolveCaches() override { return false; }

 private:
  int& _destructions;
};

//! Whether an optimizer accepts an algorithm passed as the given type.
template <typename Algorithm>
concept OptimizerAccepts =
    requires(g2o::SparseOptimizer& optimizer, Algorithm algorithm) {
      optimizer.setAlgorithm(std::move(algorithm));
    };

}  // namespace

// Interfaces that take ownership accept only std::unique_ptr, so that a
// caller can neither leak nor free the object it hands over.
static_assert(
    std::is_constructible_v<BlockSolver, std::unique_ptr<LinearSolver>>);
static_assert(std::is_constructible_v<g2o::OptimizationAlgorithmGaussNewton,
                                      std::unique_ptr<g2o::Solver>>);
static_assert(std::is_constructible_v<g2o::OptimizationAlgorithmLevenberg,
                                      std::unique_ptr<g2o::Solver>>);
static_assert(std::is_constructible_v<g2o::OptimizationAlgorithmDogleg,
                                      std::unique_ptr<g2o::BlockSolverBase>>);
static_assert(OptimizerAccepts<std::unique_ptr<g2o::OptimizationAlgorithm>>);
static_assert(!std::is_constructible_v<BlockSolver, LinearSolver*>);
static_assert(!std::is_constructible_v<g2o::OptimizationAlgorithmGaussNewton,
                                       g2o::Solver*>);
static_assert(!std::is_constructible_v<g2o::OptimizationAlgorithmLevenberg,
                                       g2o::Solver*>);
static_assert(!std::is_constructible_v<g2o::OptimizationAlgorithmDogleg,
                                       g2o::BlockSolverBase*>);
static_assert(!OptimizerAccepts<g2o::OptimizationAlgorithm*>);
static_assert(
    std::is_same_v<
        decltype(std::declval<g2o::OptimizationAlgorithmFactory&>().construct(
            std::declval<const std::string&>(),
            std::declval<g2o::OptimizationAlgorithmProperty&>())),
        std::unique_ptr<g2o::OptimizationAlgorithm>>);
static_assert(
    std::is_same_v<
        decltype(std::declval<g2o::AbstractOptimizationAlgorithmCreator&>()
                     .construct()),
        std::unique_ptr<g2o::OptimizationAlgorithm>>);

//! Whether an edge accepts a robust kernel passed as the given type.
template <typename Kernel>
concept EdgeAccepts =
    requires(g2o::OptimizableGraph::Edge& edge, Kernel kernel) {
      edge.setRobustKernel(std::move(kernel));
    };

static_assert(EdgeAccepts<std::unique_ptr<g2o::RobustKernel>>);
static_assert(!EdgeAccepts<g2o::RobustKernel*>);
static_assert(
    std::is_same_v<decltype(std::declval<g2o::RobustKernelFactory&>().construct(
                       std::declval<const std::string&>())),
                   std::unique_ptr<g2o::RobustKernel>>);
static_assert(
    std::is_same_v<
        decltype(std::declval<g2o::AbstractRobustKernelCreator&>().construct()),
        std::unique_ptr<g2o::RobustKernel>>);

//! Whether a vertex accepts user data passed as the given type.
template <typename Data>
concept VertexAcceptsUserData = requires(g2o::VertexSE2& vertex, Data data) {
  vertex.setUserData(std::move(data));
} || requires(g2o::VertexSE2& vertex, Data data) {
  vertex.addUserData(std::move(data));
};

//! Whether a graph accepts a parameter passed as the given type.
template <typename Parameter>
concept GraphAcceptsParameter =
    requires(g2o::SparseOptimizer& graph, Parameter parameter) {
      graph.addParameter(std::move(parameter));
    };

static_assert(VertexAcceptsUserData<std::unique_ptr<g2o::HyperGraph::Data>>);
static_assert(!VertexAcceptsUserData<g2o::HyperGraph::Data*>);
static_assert(GraphAcceptsParameter<std::unique_ptr<g2o::Parameter>>);
static_assert(!GraphAcceptsParameter<g2o::Parameter*>);
static_assert(std::is_same_v<decltype(std::declval<g2o::ParameterContainer&>()
                                          .detachParameter(0)),
                             std::unique_ptr<g2o::Parameter>>);

//! Whether a graph accepts a vertex passed as the given type.
template <typename Vertex>
concept GraphAcceptsVertex =
    requires(g2o::SparseOptimizer& graph, Vertex vertex) {
      graph.addVertex(std::move(vertex));
    };

//! Whether a graph accepts an edge passed as the given type.
template <typename Edge>
concept GraphAcceptsEdge = requires(g2o::SparseOptimizer& graph, Edge edge) {
  graph.addEdge(std::move(edge));
};

//! Whether the factory accepts a creator passed as the given type.
template <typename Creator>
concept FactoryAcceptsCreator =
    requires(g2o::Factory& factory, Creator creator) {
      factory.registerType(std::string{}, std::move(creator));
    };

static_assert(GraphAcceptsVertex<std::unique_ptr<g2o::VertexSE3>>);
static_assert(!GraphAcceptsVertex<g2o::VertexSE3*>);
static_assert(!GraphAcceptsVertex<g2o::HyperGraph::Vertex*>);
static_assert(GraphAcceptsEdge<std::unique_ptr<g2o::EdgeSE3PointXYZ>>);
static_assert(!GraphAcceptsEdge<g2o::EdgeSE3PointXYZ*>);
static_assert(!GraphAcceptsEdge<g2o::HyperGraph::Edge*>);

//! Whether the base graph accepts a vertex passed as the given type.
template <typename Vertex>
concept HyperGraphAcceptsVertex =
    requires(g2o::HyperGraph& graph, Vertex vertex) {
      graph.addVertex(std::move(vertex));
    };

//! Whether the base graph accepts an edge passed as the given type.
template <typename Edge>
concept HyperGraphAcceptsEdge = requires(g2o::HyperGraph& graph, Edge edge) {
  graph.addEdge(std::move(edge));
};

//! Whether the action library accepts an action passed as the given type.
template <typename Action>
concept LibraryAcceptsAction =
    requires(g2o::HyperGraphActionLibrary& library, Action action) {
      library.registerAction(std::move(action));
    };

static_assert(HyperGraphAcceptsVertex<std::unique_ptr<g2o::VertexSE3>>);
static_assert(!HyperGraphAcceptsVertex<g2o::HyperGraph::Vertex*>);
static_assert(HyperGraphAcceptsEdge<std::unique_ptr<g2o::EdgeSE3PointXYZ>>);
static_assert(!HyperGraphAcceptsEdge<g2o::HyperGraph::Edge*>);
static_assert(
    LibraryAcceptsAction<std::unique_ptr<g2o::HyperGraphElementAction>>);
static_assert(!LibraryAcceptsAction<g2o::HyperGraphElementAction*>);
static_assert(FactoryAcceptsCreator<
              std::unique_ptr<g2o::AbstractHyperGraphElementCreator>>);
static_assert(!FactoryAcceptsCreator<g2o::AbstractHyperGraphElementCreator*>);
static_assert(
    std::is_same_v<decltype(std::declval<g2o::Factory&>().construct(
                       std::declval<const std::string&>())),
                   std::unique_ptr<g2o::HyperGraph::HyperGraphElement>>);
static_assert(std::is_same_v<
              decltype(std::declval<g2o::AbstractHyperGraphElementCreator&>()
                           .construct()),
              std::unique_ptr<g2o::HyperGraph::HyperGraphElement>>);

//! Whether an action collection accepts an action passed as the given type.
template <typename Action>
concept CollectionAcceptsAction =
    requires(g2o::HyperGraphElementActionCollection& collection,
             Action action) { collection.registerAction(std::move(action)); };

static_assert(
    CollectionAcceptsAction<std::unique_ptr<g2o::HyperGraphElementAction>>);
static_assert(!CollectionAcceptsAction<g2o::HyperGraphElementAction*>);

//! Whether a property map accepts a property passed as the given type.
template <typename Property>
concept PropertyMapAccepts =
    requires(g2o::PropertyMap& properties, Property property) {
      properties.addProperty(std::move(property));
    };

static_assert(PropertyMapAccepts<std::unique_ptr<g2o::BaseProperty>>);
static_assert(!PropertyMapAccepts<g2o::BaseProperty*>);

//! Whether a data queue accepts robot data passed as the given type.
template <typename Data>
concept DataQueueAccepts =
    requires(g2o::DataQueue& queue, Data data) { queue.add(std::move(data)); };

static_assert(DataQueueAccepts<std::unique_ptr<g2o::RobotData>>);
static_assert(!DataQueueAccepts<g2o::RobotData*>);

//! An optimizer frees its algorithm when it is destroyed.
TEST(General, OptimizerDeletesItsAlgorithm) {
  int destructions = 0;
  {
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(
        std::make_unique<CountedAlgorithm>(createSolver(), destructions));
    EXPECT_EQ(destructions, 0);
    EXPECT_NE(optimizer.algorithm(), nullptr);
  }

  EXPECT_EQ(destructions, 1);
}

//! An optimizer frees the algorithm that another algorithm replaces.
TEST(General, OptimizerDeletesTheAlgorithmItReplaces) {
  int replacedDestructions = 0;
  int replacingDestructions = 0;
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(
      std::make_unique<CountedAlgorithm>(createSolver(), replacedDestructions));
  auto replacing =
      std::make_unique<CountedAlgorithm>(createSolver(), replacingDestructions);
  const CountedAlgorithm* replacingPointer = replacing.get();

  optimizer.setAlgorithm(std::move(replacing));

  EXPECT_EQ(replacedDestructions, 1);
  EXPECT_EQ(replacingDestructions, 0);
  EXPECT_EQ(optimizer.algorithm(), replacingPointer);
}

//! A vertex frees its user data and the data chained to it.
TEST(General, VertexDeletesItsUserData) {
  int destructions = 0;
  {
    g2o::VertexSE2 vertex;
    vertex.addUserData(std::make_unique<CountedData>(destructions));
    vertex.addUserData(std::make_unique<CountedData>(destructions));
    EXPECT_EQ(destructions, 0);
    ASSERT_NE(vertex.userData(), nullptr);
    EXPECT_NE(vertex.userData()->next(), nullptr);
  }

  EXPECT_EQ(destructions, 2);
}

//! A vertex frees the user data that other user data replaces.
TEST(General, VertexDeletesTheUserDataItReplaces) {
  int replacedDestructions = 0;
  int replacingDestructions = 0;
  g2o::VertexSE2 vertex;
  vertex.setUserData(std::make_unique<CountedData>(replacedDestructions));
  auto replacing = std::make_unique<CountedData>(replacingDestructions);
  const CountedData* replacingPointer = replacing.get();

  vertex.setUserData(std::move(replacing));

  EXPECT_EQ(replacedDestructions, 1);
  EXPECT_EQ(replacingDestructions, 0);
  EXPECT_EQ(vertex.userData(), replacingPointer);
}

//! A graph frees its parameters when it is destroyed.
TEST(General, GraphDeletesItsParameters) {
  int destructions = 0;
  {
    g2o::SparseOptimizer graph;
    ASSERT_TRUE(graph.addParameter(
        std::make_unique<CountedParameter>(0, destructions)));
    EXPECT_EQ(destructions, 0);
    EXPECT_NE(graph.parameter(0), nullptr);
  }

  EXPECT_EQ(destructions, 1);
}

//! A graph frees a parameter whose id another parameter already has.
TEST(General, GraphDeletesARejectedParameter) {
  int keptDestructions = 0;
  int rejectedDestructions = 0;
  g2o::SparseOptimizer graph;
  auto kept = std::make_unique<CountedParameter>(0, keptDestructions);
  const CountedParameter* keptPointer = kept.get();
  ASSERT_TRUE(graph.addParameter(std::move(kept)));

  EXPECT_FALSE(graph.addParameter(
      std::make_unique<CountedParameter>(0, rejectedDestructions)));

  EXPECT_EQ(keptDestructions, 0);
  EXPECT_EQ(rejectedDestructions, 1);
  EXPECT_EQ(graph.parameter(0), keptPointer);
}

//! A graph frees a vertex whose id another vertex already has.
TEST(General, GraphDeletesARejectedVertex) {
  int keptDestructions = 0;
  int rejectedDestructions = 0;
  g2o::SparseOptimizer graph;
  auto kept = std::make_unique<CountedVertex>(0, keptDestructions);
  const CountedVertex* keptPointer = kept.get();
  ASSERT_TRUE(graph.addVertex(std::move(kept)));

  EXPECT_FALSE(graph.addVertex(
      std::make_unique<CountedVertex>(0, rejectedDestructions)));

  EXPECT_EQ(keptDestructions, 0);
  EXPECT_EQ(rejectedDestructions, 1);
  EXPECT_EQ(graph.vertex(0), keptPointer);
  EXPECT_EQ(graph.vertices().size(), 1U);
}

/**
 * A graph frees an edge whose parameters it lacks and keeps no reference to
 * it.
 */
TEST(General, GraphDeletesAnEdgeWithoutItsParameter) {
  int destructions = 0;
  g2o::SparseOptimizer graph;
  auto pose = std::make_unique<g2o::VertexSE3>();
  pose->setId(0);
  g2o::VertexSE3* posePointer = pose.get();
  ASSERT_TRUE(graph.addVertex(std::move(pose)));
  auto point = std::make_unique<g2o::VertexPointXYZ>();
  point->setId(1);
  g2o::VertexPointXYZ* pointPointer = point.get();
  ASSERT_TRUE(graph.addVertex(std::move(point)));
  auto edge = std::make_unique<CountedEdge>(destructions);
  edge->setVertex(0, posePointer);
  edge->setVertex(1, pointPointer);
  constexpr int missingParameterId = 0;
  ASSERT_TRUE(edge->setParameterId(0, missingParameterId));

  EXPECT_FALSE(graph.addEdge(std::move(edge)));

  EXPECT_EQ(destructions, 1);
  EXPECT_TRUE(graph.edges().empty());
  EXPECT_TRUE(posePointer->edges().empty());
  EXPECT_TRUE(pointPointer->edges().empty());
}

//! A property map frees a property whose name another property already has.
TEST(General, PropertyMapDeletesARejectedProperty) {
  int keptDestructions = 0;
  int rejectedDestructions = 0;
  g2o::PropertyMap properties;
  auto kept = std::make_unique<CountedProperty>("name", keptDestructions);
  const CountedProperty* keptPointer = kept.get();
  ASSERT_TRUE(properties.addProperty(std::move(kept)));

  EXPECT_FALSE(properties.addProperty(
      std::make_unique<CountedProperty>("name", rejectedDestructions)));

  EXPECT_EQ(keptDestructions, 0);
  EXPECT_EQ(rejectedDestructions, 1);
  EXPECT_EQ(properties.getProperty<CountedProperty>("name"), keptPointer);
  EXPECT_EQ(properties.size(), 1U);
}

//! A data queue frees its data when it is destroyed.
TEST(General, DataQueueDeletesItsData) {
  int destructions = 0;
  {
    g2o::DataQueue queue;
    queue.add(std::make_unique<CountedRobotData>(1., destructions));
    queue.add(std::make_unique<CountedRobotData>(2., destructions));
    EXPECT_EQ(destructions, 0);
    EXPECT_EQ(queue.buffer().size(), 2U);
  }

  EXPECT_EQ(destructions, 2);
}

//! A data queue frees the data that data with the same timestamp replaces.
TEST(General, DataQueueDeletesReplacedData) {
  int replacedDestructions = 0;
  int replacingDestructions = 0;
  g2o::DataQueue queue;
  queue.add(std::make_unique<CountedRobotData>(1., replacedDestructions));
  auto replacing =
      std::make_unique<CountedRobotData>(1., replacingDestructions);
  const CountedRobotData* replacingPointer = replacing.get();

  queue.add(std::move(replacing));

  EXPECT_EQ(replacedDestructions, 1);
  EXPECT_EQ(replacingDestructions, 0);
  EXPECT_EQ(queue.buffer().at(1.).get(), replacingPointer);
  EXPECT_EQ(queue.buffer().size(), 1U);
}

//! A hyper graph keeps a vertex it owns already when it is added again.
TEST(General, HyperGraphKeepsAVertexAddedTwice) {
  int destructions = 0;
  g2o::HyperGraph graph;
  auto vertex = std::make_unique<CountedVertex>(0, destructions);
  CountedVertex* added = vertex.get();
  ASSERT_TRUE(graph.addVertex(std::move(vertex)));

  EXPECT_FALSE(
      graph.addVertex(std::unique_ptr<g2o::HyperGraph::Vertex>(added)));

  EXPECT_EQ(destructions, 0);
  EXPECT_EQ(graph.vertex(0), added);
  EXPECT_EQ(graph.vertices().size(), 1U);
}

//! A graph frees an edge whose caches do not resolve and keeps no reference.
TEST(General, GraphDeletesAnEdgeWithoutItsCaches) {
  int destructions = 0;
  g2o::SparseOptimizer graph;
  for (int id = 0; id < 2; ++id) {
    auto vertex = std::make_unique<g2o::VertexSE2>();
    vertex->setId(id);
    ASSERT_TRUE(graph.addVertex(std::move(vertex)));
  }
  auto edge = std::make_unique<EdgeWithoutCaches>(destructions);
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));

  EXPECT_FALSE(graph.addEdge(std::move(edge)));

  EXPECT_EQ(destructions, 1);
  EXPECT_TRUE(graph.edges().empty());
  EXPECT_TRUE(graph.vertex(0)->edges().empty());
  EXPECT_TRUE(graph.vertex(1)->edges().empty());
}
