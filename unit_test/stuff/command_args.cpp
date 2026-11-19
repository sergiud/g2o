// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/command_args.h"

#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

namespace {

//! Parses the given arguments after the name of a program.
bool parse(g2o::CommandArgs& arguments, std::vector<std::string> values) {
  values.insert(values.begin(), "program");
  std::vector<char*> argv;
  for (std::string& value : values) {
    argv.push_back(value.data());
  }
  argv.push_back(nullptr);
  return arguments.parseArgs(static_cast<int>(values.size()), argv.data(),
                             /*exitOnError=*/false);
}

}  // namespace

//! A value that is no number leaves the default of an option in place.
TEST(CommandArgs, RejectsAValueThatIsNoNumber) {
  g2o::CommandArgs arguments;
  int iterations = 0;
  arguments.param("i", iterations, 5, "iterations");

  EXPECT_FALSE(parse(arguments, {"-i", "abc"}));
}

//! A number followed by more characters is no number either.
TEST(CommandArgs, RejectsTrailingCharacters) {
  g2o::CommandArgs arguments;
  int iterations = 0;
  arguments.param("i", iterations, 5, "iterations");

  EXPECT_FALSE(parse(arguments, {"-i", "12abc"}));
}

TEST(CommandArgs, ParsesAListOfIntegers) {
  g2o::CommandArgs arguments;
  std::vector<int> ids;
  arguments.param("ids", ids, {}, "ids");

  ASSERT_TRUE(parse(arguments, {"-ids", "1,2,3,11,12,13,21,22,23,31,32,33"}));

  EXPECT_THAT(ids, ElementsAre(1, 2, 3, 11, 12, 13, 21, 22, 23, 31, 32, 33));
}

TEST(CommandArgs, ParsesAListOfDoubles) {
  g2o::CommandArgs arguments;
  std::vector<double> noise;
  arguments.param("noise", noise, {}, "noise");

  ASSERT_TRUE(parse(arguments, {"-noise", "0.1;0.2;0.3"}));

  EXPECT_THAT(noise, ElementsAre(0.1, 0.2, 0.3));
}

//! An element that is no number makes the whole list invalid.
TEST(CommandArgs, RejectsAListWithAnInvalidElement) {
  for (const std::string list : {"1,2,x", "1,2x", "1,,2"}) {
    g2o::CommandArgs arguments;
    std::vector<int> ids;
    arguments.param("ids", ids, {}, "ids");

    EXPECT_FALSE(parse(arguments, {"-ids", list})) << list;
  }
}
