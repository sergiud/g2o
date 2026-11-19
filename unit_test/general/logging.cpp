// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <ng-log/experimental/setup.h>
#include <ng-log/experimental/sink.h>

#include <memory>
#include <string>
#include <vector>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/types/slam3d/edge_se3_pointxyz.h"
#include "g2o/types/slam3d/parameter_se3_offset.h"
#include "g2o/types/slam3d/vertex_pointxyz.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::AllOf;
using ::testing::Contains;
using ::testing::ElementsAre;
using ::testing::Field;
using ::testing::HasSubstr;

namespace {

namespace ngx = nglog::experimental;

//! Message that a structured sink received.
struct LoggedMessage {
  nglog::LogSeverity severity;
  std::string text;
};

//! Structured sink that keeps the messages it receives while registered.
class CapturingSink final : public ngx::Sink {
 public:
  CapturingSink() { ngx::AddSink(*this); }
  ~CapturingSink() { ngx::RemoveSink(*this); }

  CapturingSink(const CapturingSink&) = delete;
  CapturingSink& operator=(const CapturingSink&) = delete;

  void Send(const ngx::Record& record) override {
    messages.push_back(
        LoggedMessage{record.severity(), std::string{record.text()}});
  }

  std::vector<LoggedMessage> messages;
};

}  // namespace

/**
 * The library reports its diagnostics through ng-log, so that applications
 * decide where they go, rather than writing them to the standard error.
 */
TEST(General, OptimizeWithoutVerticesLogsAnError) {
  CapturingSink sink;
  g2o::SparseOptimizer optimizer;

  EXPECT_EQ(optimizer.optimize(1), -1);

  EXPECT_THAT(sink.messages,
              ElementsAre(AllOf(
                  Field("severity", &LoggedMessage::severity, NGLOG_ERROR),
                  Field("text", &LoggedMessage::text,
                        HasSubstr("0 vertices to optimize")))));
}

//! Error that the graph logs for the edge between the vertices 3 and 7.
auto ParameterErrorOfEdge() {
  return Contains(AllOf(
      Field("severity", &LoggedMessage::severity, NGLOG_ERROR),
      Field("text", &LoggedMessage::text,
            AllOf(HasSubstr("cannot resolve parameters"), HasSubstr("3 7")))));
}

/**
 * An edge whose parameter is missing from the graph is rejected, and the
 * error names the vertices of the edge.
 */
TEST(General, EdgeWithoutItsParameterLogsAnError) {
  CapturingSink sink;
  g2o::SparseOptimizer optimizer;
  auto pose = std::make_unique<g2o::VertexSE3>();
  pose->setId(3);
  auto point = std::make_unique<g2o::VertexPointXYZ>();
  point->setId(7);
  auto edge = std::make_unique<g2o::EdgeSE3PointXYZ>();
  edge->setVertex(0, pose.get());
  edge->setVertex(1, point.get());
  ASSERT_TRUE(edge->setParameterId(0, 0));
  ASSERT_TRUE(optimizer.addVertex(std::move(pose)));
  ASSERT_TRUE(optimizer.addVertex(std::move(point)));

  EXPECT_FALSE(optimizer.addEdge(std::move(edge)));

  EXPECT_THAT(sink.messages, ParameterErrorOfEdge());
}

/**
 * Reconnecting a vertex of an edge resolves its parameters again, which
 * fails the same way for a missing parameter.
 */
TEST(General, ReconnectingAnEdgeWithoutItsParameterLogsAnError) {
  constexpr int MissingParameterId = 5;
  g2o::SparseOptimizer optimizer;
  auto offset = std::make_unique<g2o::ParameterSE3Offset>();
  offset->setId(0);
  ASSERT_TRUE(optimizer.addParameter(std::move(offset)));
  auto pose = std::make_unique<g2o::VertexSE3>();
  pose->setId(3);
  auto point = std::make_unique<g2o::VertexPointXYZ>();
  point->setId(7);
  auto edge = std::make_unique<g2o::EdgeSE3PointXYZ>();
  auto* reconnected = edge.get();
  auto* last = point.get();
  edge->setVertex(0, pose.get());
  edge->setVertex(1, point.get());
  ASSERT_TRUE(edge->setParameterId(0, 0));
  ASSERT_TRUE(optimizer.addVertex(std::move(pose)));
  ASSERT_TRUE(optimizer.addVertex(std::move(point)));
  ASSERT_TRUE(optimizer.addEdge(std::move(edge)));
  ASSERT_TRUE(reconnected->setParameterId(0, MissingParameterId));
  CapturingSink sink;

  EXPECT_FALSE(optimizer.setEdgeVertex(reconnected, 1, last));

  EXPECT_THAT(sink.messages, ParameterErrorOfEdge());
}
