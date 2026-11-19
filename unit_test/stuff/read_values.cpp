// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/read_values.h"

#include <cstddef>
#include <sstream>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

namespace {

constexpr std::size_t Maximum = 4;

}  // namespace

TEST(ReadCount, AcceptsTheMaximum) {
  std::istringstream is("4");
  std::size_t count = 0;

  EXPECT_TRUE(g2o::readCount(is, count, Maximum));
  EXPECT_EQ(count, Maximum);
}

//! A count above the maximum would overflow the sizes it determines.
TEST(ReadCount, RejectsACountAboveTheMaximum) {
  std::istringstream is("5");
  std::size_t count = 0;

  EXPECT_FALSE(g2o::readCount(is, count, Maximum));
  EXPECT_TRUE(is.fail());
}

TEST(ReadCount, RejectsANegativeCount) {
  std::istringstream is("-1");
  std::size_t count = 0;

  EXPECT_FALSE(g2o::readCount(is, count, Maximum));
  EXPECT_TRUE(is.fail());
}

TEST(ReadValues, AppendsTheValues) {
  std::istringstream is("1.5 2.5 3.5");
  std::vector<double> values{0.5};

  EXPECT_TRUE(g2o::readValues(is, 2, values));
  EXPECT_THAT(values, ElementsAre(0.5, 1.5, 2.5));
}

TEST(ReadValues, FailsAtTheEndOfTheInput) {
  std::istringstream is("1.5 2.5");
  std::vector<double> values;

  EXPECT_FALSE(g2o::readValues(is, 3, values));
}
