// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o_interactive/g2o_slam_interface.h>
#include <g2o_interactive/graph_optimizer_sparse_online.h>
#include <g2o_interactive/types_slam3d_online.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::IsEmpty;
using ::testing::SizeIs;

namespace {

//! Dimension of the poses and the edges of a single SLAM scenario.
struct Scenario {
  std::string tag;
  int dimension;
  std::size_t measurementSize;
  std::size_t informationSize;
};

const Scenario Planar{"EDGE_XYT", 3, 3, 6};
const Scenario Spatial{"EDGE_XYZRPY", 6, 6, 21};

void PrintTo(const Scenario& scenario, std::ostream* os) {
  *os << scenario.tag;
}

//! Scenario whose poses differ in dimension from those of the given one.
const Scenario& otherScenario(const Scenario& scenario) {
  return scenario.dimension == Planar.dimension ? Spatial : Planar;
}

class SlamInterfaceEdges : public ::testing::TestWithParam<Scenario> {
 protected:
  //! Adds an edge with an identity measurement and unit information.
  bool addEdge(int from, int to) { return addEdge(GetParam(), from, to); }

  bool addEdge(const Scenario& scenario, int from, int to) {
    std::vector<double> information(scenario.informationSize, 0.);
    // The information is stored as the upper triangle row by row, whose
    // diagonal entries start each row.
    std::size_t index = 0;
    for (int row = scenario.dimension; row > 0; --row) {
      information[index] = 1.;
      index += row;
    }
    return slam_.addEdge(scenario.tag, 0, scenario.dimension, from, to,
                         std::vector<double>(scenario.measurementSize, 0.),
                         information);
  }

  g2o::SparseOptimizerOnline optimizer_;
  g2o::G2oSlamInterface slam_{&optimizer_};
};

}  // namespace

/**
 * The graph rejects a vertex with a negative id and destroys it, so the edge
 * that would connect to it must be rejected as well.
 */
TEST_P(SlamInterfaceEdges, RejectsANegativeVertexId) {
  ASSERT_TRUE(slam_.addNode(GetParam().tag, 0, GetParam().dimension, {}));

  EXPECT_FALSE(addEdge(0, -1));
  EXPECT_THAT(optimizer_.edges(), IsEmpty());
}

/**
 * An edge from a new vertex to itself would add the vertex twice, and the
 * graph destroys the duplicate.
 */
TEST_P(SlamInterfaceEdges, RejectsAnEdgeFromAVertexToItself) {
  ASSERT_TRUE(slam_.addNode(GetParam().tag, 5, GetParam().dimension, {}));

  EXPECT_FALSE(addEdge(5, 5));
  EXPECT_THAT(optimizer_.edges(), IsEmpty());
}

/**
 * A vertex that already exists cannot be connected to itself either, since
 * the relative measurement of an edge needs two distinct poses.
 */
TEST_P(SlamInterfaceEdges, RejectsAnEdgeFromAnExistingVertexToItself) {
  ASSERT_TRUE(slam_.addNode(GetParam().tag, 4, GetParam().dimension, {}));
  ASSERT_TRUE(addEdge(4, 5));

  EXPECT_FALSE(addEdge(5, 5));
  EXPECT_THAT(optimizer_.edges(), SizeIs(1));
}

/**
 * The solver and the vertex types follow the dimension of the first pose, so
 * an edge between poses of another dimension cannot join the graph.
 */
TEST_P(SlamInterfaceEdges, RejectsAnEdgeOfAnotherDimension) {
  ASSERT_TRUE(slam_.addNode(GetParam().tag, 0, GetParam().dimension, {}));
  ASSERT_TRUE(addEdge(0, 1));

  EXPECT_FALSE(addEdge(otherScenario(GetParam()), 1, 2));
  EXPECT_FALSE(addEdge(otherScenario(GetParam()), 0, 1));
  EXPECT_THAT(optimizer_.edges(), SizeIs(1));
  slam_.setUpdateGraphEachN(1);
  EXPECT_TRUE(slam_.solveState());
}

/**
 * The protocol declares poses with an ADD VERTEX command, which a client may
 * omit since the edges introduce their poses anyway.
 */
TEST_P(SlamInterfaceEdges, SolvesAGraphOfEdgesAlone) {
  slam_.setUpdateGraphEachN(1);
  ASSERT_TRUE(addEdge(0, 1));

  EXPECT_TRUE(slam_.solveState());
}

/**
 * A pose of a dimension without a solver must not decide the solver of the
 * poses that follow.
 */
TEST_P(SlamInterfaceEdges, IgnoresAPoseOfAnUnsupportedDimension) {
  constexpr int UnsupportedDimension = 4;
  EXPECT_FALSE(slam_.addNode("VERTEX_XYZ", 0, UnsupportedDimension, {}));

  EXPECT_TRUE(addEdge(0, 1));
}

INSTANTIATE_TEST_SUITE_P(InteractiveSlam, SlamInterfaceEdges,
                         ::testing::Values(Planar, Spatial),
                         [](const ::testing::TestParamInfo<Scenario>& info) {
                           return info.param.tag;
                         });

/**
 * An edge in Euler angles converts its information into the minimal
 * quaternion parameters of the pose, whose last component is sin(yaw/2) for
 * a rotation about z, so the derivative of the yaw is 2/cos(yaw/2).
 */
TEST(InteractiveSlam, ConvertsTheInformationOfTheYaw) {
  constexpr double Yaw = 3.;
  constexpr double Tolerance = 1e-6;
  g2o::SparseOptimizerOnline optimizer;
  g2o::G2oSlamInterface slam(&optimizer);
  std::vector<double> information(Spatial.informationSize, 0.);
  std::size_t index = 0;
  for (int row = Spatial.dimension; row > 0; --row) {
    information[index] = 1.;
    index += row;
  }
  ASSERT_TRUE(slam.addEdge(Spatial.tag, 0, Spatial.dimension, 0, 1,
                           {0., 0., 0., 0., 0., Yaw}, information));

  ASSERT_THAT(optimizer.edges(), SizeIs(1));
  const auto* edge = static_cast<const g2o::OptimizableGraph::Edge*>(
      optimizer.edges().begin()->get());
  const double derivative = 2. / std::cos(Yaw / 2.);
  EXPECT_THAT(edge->informationData()[5 * Spatial.dimension + 5],
              ::testing::DoubleNear(derivative * derivative,
                                    Tolerance * derivative * derivative));
}

/**
 * The protocol lists the poses of a query by their ids in ascending order,
 * which a client reads in that order.
 */
TEST(InteractiveSlam, QueriesThePosesInTheOrderOfTheirIds) {
  constexpr int Poses = 16;
  g2o::SparseOptimizerOnline optimizer;
  g2o::G2oSlamInterface slam(&optimizer);
  const std::vector<double> information{1., 0., 0., 1., 0., 1.};
  for (int id = 0; id + 1 < Poses; ++id) {
    ASSERT_TRUE(slam.addEdge(Planar.tag, 0, Planar.dimension, id, id + 1,
                             {1., 0., 0.}, information));
  }

  ::testing::internal::CaptureStdout();
  ASSERT_TRUE(slam.queryState({}));
  std::istringstream output(::testing::internal::GetCapturedStdout());

  std::vector<int> ids;
  for (std::string line; std::getline(output, line);) {
    std::istringstream fields(line);
    std::string tag;
    int id = -1;
    if (fields >> tag >> id && tag == "VERTEX_XYT") {
      ids.push_back(id);
    }
  }
  ASSERT_THAT(ids, SizeIs(Poses));
  EXPECT_TRUE(std::ranges::is_sorted(ids)) << ::testing::PrintToString(ids);
}

/**
 * The online optimization keeps its result in the updated estimates, which
 * the error of an edge reports in the space of poses as in the plane.
 */
TEST(InteractiveSlam, Chi2OfASpatialEdgeUsesTheUpdatedEstimates) {
  g2o::OnlineVertexSE3 from;
  g2o::OnlineVertexSE3 to;
  Eigen::Isometry3d moved = Eigen::Isometry3d::Identity();
  moved.translation() = Eigen::Vector3d(1.5, 0., 0.);
  to.updatedEstimate = moved;
  g2o::OnlineEdgeSE3 edge;
  edge.setVertex(0, &from);
  edge.setVertex(1, &to);
  Eigen::Isometry3d measurement = Eigen::Isometry3d::Identity();
  measurement.translation() = Eigen::Vector3d(1., 0., 0.);
  edge.setMeasurement(measurement);
  edge.setInformation(g2o::EdgeSE3::InformationType::Identity());

  EXPECT_DOUBLE_EQ(edge.chi2(), 0.25);
}
