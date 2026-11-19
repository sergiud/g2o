// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/factory.h"

#include <array>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/data/vertex_tag.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "g2o/types/sim3/types_seven_dof_expmap.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam3d/edge_se3.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::AllOf;
using ::testing::AnyOf;
using ::testing::DistanceFrom;
using ::testing::Each;
using ::testing::HasSubstr;
using ::testing::Le;
using ::testing::Not;
using ::testing::NotNull;
using ::testing::SizeIs;

/**
 * The listing of the registered types is output for the caller, so every
 * type name goes to the stream the caller passes.
 */
TEST(General, FactoryPrintsRegisteredTypesToTheGivenStream) {
  // A vertex of the library makes sure that its types are registered.
  g2o::VertexSE2 vertex;
  std::ostringstream listing;

  g2o::Factory::instance()->printRegisteredTypes(listing, true);

  EXPECT_THAT(listing.str(), HasSubstr("#\tVERTEX_SE2\n"));
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

  // The tags with a colon are the ones that formatting can break.
  EXPECT_THAT(listing.str(), AllOf(HasSubstr("\tVERTEX_SE3:QUAT\n"),
                                   HasSubstr("\tVERTEX_SE3:EXPMAP\n"),
                                   HasSubstr("\tVERTEX_SIM3:EXPMAP\n")));
  std::istringstream lines(listing.str());
  std::string line;
  std::getline(lines, line);  // The heading of the listing.
  std::vector<std::string> tags;
  while (std::getline(lines, line)) {
    tags.push_back(line.substr(1));
  }
  EXPECT_THAT(tags, Each(Not(AnyOf(HasSubstr(" "), HasSubstr("\t")))));
}

/**
 * A graph of 3D poses written by save() loads into a graph with the same
 * vertices and edges.
 */
TEST(General, GraphOfSE3PosesSurvivesSaveAndLoad) {
  // The precision of the numbers written to the file.
  constexpr double precision = 1e-6;
  const std::array<Eigen::Isometry3d, 2> poses = {
      Eigen::Translation3d(1., 2., 3.) *
          Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ()),
      Eigen::Translation3d(-2., 0.5, 1.) *
          Eigen::AngleAxisd(-0.7, Eigen::Vector3d::UnitX())};
  const Eigen::Isometry3d measurement =
      Eigen::Translation3d(0.1, -0.2, 0.3) *
      Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitY());
  g2o::SparseOptimizer graph;
  for (int id = 0; id < 2; ++id) {
    auto vertex = std::make_unique<g2o::VertexSE3>();
    vertex->setId(id);
    vertex->setEstimate(poses[id]);
    ASSERT_TRUE(graph.addVertex(std::move(vertex)));
  }
  auto edge = std::make_unique<g2o::EdgeSE3>();
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));
  edge->setMeasurement(measurement);
  ASSERT_TRUE(graph.addEdge(std::move(edge)));
  std::stringstream file;
  ASSERT_TRUE(graph.save(file));

  g2o::SparseOptimizer loaded;
  ASSERT_TRUE(loaded.load(file));

  ASSERT_THAT(loaded.vertices(), SizeIs(2));
  ASSERT_THAT(loaded.edges(), SizeIs(1));
  for (int id = 0; id < 2; ++id) {
    const auto* vertex = dynamic_cast<const g2o::VertexSE3*>(loaded.vertex(id));
    ASSERT_THAT(vertex, NotNull());
    EXPECT_THAT(vertex->estimate().matrix(),
                DistanceFrom(poses[id].matrix(), g2o::test::MaxAbsDifference,
                             Le(precision)));
  }
  const auto* loadedEdge =
      dynamic_cast<const g2o::EdgeSE3*>(loaded.edges().begin()->get());
  ASSERT_THAT(loadedEdge, NotNull());
  EXPECT_EQ(loadedEdge->vertex(0)->id(), 0);
  EXPECT_EQ(loadedEdge->vertex(1)->id(), 1);
  EXPECT_THAT(loadedEdge->measurement().matrix(),
              DistanceFrom(measurement.matrix(), g2o::test::MaxAbsDifference,
                           Le(precision)));
}

/**
 * A saved graph is read back with the values it held, since a map in
 * projected coordinates of several kilometers needs more than the six
 * significant digits a stream writes by default.
 */
TEST(General, GraphOfSE2PosesSurvivesSaveAndLoadExactly) {
  const g2o::SE2 pose(12345.678901234567, -9876.5432109876543,
                      0.12345678901234567);
  const g2o::SE2 measurement(1.0000012345678901, -0.98765432109876543,
                             0.0012345678901234567);
  const g2o::SE2 offset(0.12345678901234567, 0.98765432109876543,
                        0.31415926535897931);
  g2o::SparseOptimizer graph;
  auto parameter = std::make_unique<g2o::ParameterSE2Offset>();
  parameter->setId(0);
  parameter->setOffset(offset);
  ASSERT_TRUE(graph.addParameter(std::move(parameter)));
  for (int id = 0; id < 2; ++id) {
    auto vertex = std::make_unique<g2o::VertexSE2>();
    vertex->setId(id);
    vertex->setEstimate(id == 0 ? pose : pose * measurement);
    ASSERT_TRUE(graph.addVertex(std::move(vertex)));
  }
  auto edge = std::make_unique<g2o::EdgeSE2>();
  edge->setVertex(0, graph.vertex(0));
  edge->setVertex(1, graph.vertex(1));
  edge->setMeasurement(measurement);
  ASSERT_TRUE(graph.addEdge(std::move(edge)));
  std::stringstream file;
  ASSERT_TRUE(graph.save(file));

  g2o::SparseOptimizer loaded;
  ASSERT_TRUE(loaded.load(file));

  const auto* vertex = dynamic_cast<const g2o::VertexSE2*>(loaded.vertex(0));
  ASSERT_THAT(vertex, NotNull());
  EXPECT_EQ(vertex->estimate().toVector(), pose.toVector());
  ASSERT_THAT(loaded.edges(), SizeIs(1));
  const auto* loadedEdge =
      dynamic_cast<const g2o::EdgeSE2*>(loaded.edges().begin()->get());
  ASSERT_THAT(loadedEdge, NotNull());
  EXPECT_EQ(loadedEdge->measurement().toVector(), measurement.toVector());
  const auto* loadedParameter =
      dynamic_cast<const g2o::ParameterSE2Offset*>(loaded.parameter(0));
  ASSERT_THAT(loadedParameter, NotNull());
  EXPECT_EQ(loadedParameter->offset().toVector(), offset.toVector());
}

//! A parameter saved on its own keeps its values as well.
TEST(General, SavedParameterKeepsItsValues) {
  const g2o::SE2 offset(0.12345678901234567, 0.98765432109876543,
                        0.31415926535897931);
  g2o::ParameterSE2Offset parameter;
  parameter.setId(0);
  parameter.setOffset(offset);
  const g2o::SparseOptimizer graph;
  std::stringstream file;
  ASSERT_TRUE(graph.saveParameter(file, &parameter));

  std::string tag;
  int id = -1;
  Eigen::Vector3d values;
  file >> tag >> id >> values.x() >> values.y() >> values.z();
  EXPECT_EQ(values, offset.toVector());
}

//! The data attached to a vertex keeps its values when saved on its own.
TEST(General, SavedUserDataKeepsItsValues) {
  const g2o::Vector3F position(0.12345678f, 98765.4321f, -0.31415926f);
  g2o::VertexTag tag;
  tag.setName("landmark");
  tag.setPosition(position);
  const g2o::SparseOptimizer graph;
  std::stringstream file;
  ASSERT_TRUE(graph.saveUserData(file, &tag));

  std::string type;
  std::string name;
  g2o::Vector3F values;
  file >> type >> name >> values.x() >> values.y() >> values.z();
  EXPECT_EQ(values, position);
}
