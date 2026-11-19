// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <ng-log/experimental/setup.h>
#include <ng-log/experimental/sink.h>
#include <slam_parser/interface/abstract_slam_interface.h>
#include <slam_parser/interface/parser_interface.h>

#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::AllOf;
using ::testing::ElementsAre;
using ::testing::Field;
using ::testing::HasSubstr;
using ::testing::Return;

namespace {

class MockSlamInterface : public SlamParser::AbstractSlamInterface {
 public:
  MOCK_METHOD(bool, addNode,
              (const std::string&, int, int, const std::vector<double>&),
              (override));
  MOCK_METHOD(bool, addEdge,
              (const std::string&, int, int, int, int,
               const std::vector<double>&, const std::vector<double>&),
              (override));
  MOCK_METHOD(bool, fixNode, (const std::vector<int>&), (override));
  MOCK_METHOD(bool, queryState, (const std::vector<int>&), (override));
  MOCK_METHOD(bool, solveState, (), (override));
};

//! Number of parser calls that ends the loop of a parser stuck on its input.
constexpr int MaxCalls = 100;

//! Parses every command of the input and returns the number of calls.
int parseAll(SlamParser::ParserInterface& parser, std::istream& input) {
  int calls = 0;
  while (calls < MaxCalls && parser.parseCommand(input)) {
    ++calls;
  }
  return calls;
}

namespace ngx = nglog::experimental;

//! Message that a structured sink received.
struct LoggedMessage {
  nglog::LogSeverity severity;
  std::string text;
};

void PrintTo(const LoggedMessage& message, std::ostream* os) {
  *os << message.severity << ": " << message.text;
}

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
 * An empty command between two separators carries nothing to parse, and the
 * commands after it reach the SLAM interface.
 */
TEST(SlamParser, SkipsAnEmptyCommand) {
  MockSlamInterface slam;
  EXPECT_CALL(slam, solveState()).WillOnce(Return(true));
  EXPECT_CALL(slam, queryState(::testing::IsEmpty())).WillOnce(Return(true));
  SlamParser::ParserInterface parser(&slam);
  std::istringstream input("SOLVE_STATE;;QUERY_STATE;");

  EXPECT_THAT(parseAll(parser, input), ::testing::Lt(MaxCalls));
}

//! Input that starts with a separator holds an empty first command.
TEST(SlamParser, SkipsALeadingSeparator) {
  MockSlamInterface slam;
  EXPECT_CALL(slam, solveState()).WillOnce(Return(true));
  SlamParser::ParserInterface parser(&slam);
  std::istringstream input(";SOLVE_STATE;");

  EXPECT_THAT(parseAll(parser, input), ::testing::Lt(MaxCalls));
}

/**
 * A command the grammar rejects is reported as an error with its location,
 * so a client learns which command of its stream was lost.
 */
TEST(SlamParser, ReportsASyntaxErrorWithItsLocation) {
  MockSlamInterface slam;
  SlamParser::ParserInterface parser(&slam);
  // The separator at column 4 ends the command before its pose.
  std::istringstream input("ADD;");
  CapturingSink sink;

  parseAll(parser, input);

  EXPECT_THAT(sink.messages,
              ElementsAre(AllOf(
                  Field("severity", &LoggedMessage::severity, NGLOG_ERROR),
                  Field("text", &LoggedMessage::text,
                        AllOf(HasSubstr("1.4"), HasSubstr("syntax error"))))));
}

/**
 * A byte outside of ASCII is no token of the grammar, which reports it
 * instead of taking it for the end of the input and dropping the rest.
 */
TEST(SlamParser, ReportsAByteOutsideOfAscii) {
  MockSlamInterface slam;
  EXPECT_CALL(slam, solveState()).WillOnce(Return(true));
  SlamParser::ParserInterface parser(&slam);
  std::istringstream input("SOLVE_STATE;\xc3\xa9 QUERY_STATE;");
  CapturingSink sink;

  parseAll(parser, input);

  EXPECT_THAT(sink.messages,
              ::testing::Contains(Field("text", &LoggedMessage::text,
                                        HasSubstr("syntax error"))));
}

//! A spatial pose added with its values passes them on.
TEST(SlamParser, PassesTheValuesOfASpatialPose) {
  MockSlamInterface slam;
  EXPECT_CALL(slam, addNode("VERTEX_XYZRPY", 7, 6,
                            ElementsAre(1., 2., 3., 0.1, 0.2, 0.3)))
      .WillOnce(Return(true));
  SlamParser::ParserInterface parser(&slam);
  std::istringstream input("ADD VERTEX_XYZRPY 7 1 2 3 0.1 0.2 0.3;");

  parseAll(parser, input);
}

/**
 * An id beyond the range of an int names no vertex, which the parser
 * reports instead of adding a vertex under a wrapped id.
 */
TEST(SlamParser, ReportsAnIdOutOfRange) {
  MockSlamInterface slam;
  EXPECT_CALL(slam, addNode).Times(0);
  SlamParser::ParserInterface parser(&slam);
  std::istringstream input("ADD VERTEX_XYT 4294967297 0 0 0;");
  CapturingSink sink;

  parseAll(parser, input);

  EXPECT_THAT(sink.messages,
              ::testing::Contains(Field("text", &LoggedMessage::text,
                                        HasSubstr("syntax error"))));
}
