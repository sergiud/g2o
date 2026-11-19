// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o_interactive/fast_output.h>

#include <limits>
#include <string>

#include "gtest/gtest.h"

namespace {

std::string format(double value, int precision) {
  char buffer[64];
  const int length = modp_dtoa(value, buffer, precision);
  return std::string(buffer, length);
}

}  // namespace

/**
 * Rounding the fraction of a value halfway between two last digits may carry
 * into the whole part.
 */
TEST(FastOutput, CarriesTheRoundingIntoTheWholePart) {
  EXPECT_EQ(format(0.9999995, 6), "1.000000");
}

//! A value beyond an int is written in exponential notation.
TEST(FastOutput, WritesLargeValuesExponentially) {
  EXPECT_EQ(format(3e9, 6), "3.000000e+09");
}

//! A pose that diverged prints as such rather than ending the program.
TEST(FastOutput, WritesNaN) {
  EXPECT_EQ(format(std::numeric_limits<double>::quiet_NaN(), 6), "nan");
}
