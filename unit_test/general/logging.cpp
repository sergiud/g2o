// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <ng-log/experimental/setup.h>
#include <ng-log/experimental/sink.h>

#include <string>
#include <vector>

#include "g2o/core/sparse_optimizer.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::AllOf;
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
