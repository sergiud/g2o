// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "gm2dl_io.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "g2o/core/sparse_optimizer.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;
using ::testing::IsEmpty;
using ::testing::Not;
using ::testing::Pointwise;

namespace {

//! The numbers of the edges of a gm2dl file in their order.
std::vector<double> edges(const std::string& filename) {
  std::ifstream file(filename);
  std::vector<double> values;
  for (std::string line; std::getline(file, line);) {
    if (line.starts_with("EDGE2")) {
      std::istringstream fields(line.substr(std::string{"EDGE2"}.size()));
      for (double value = 0.; fields >> value;) {
        values.push_back(value);
      }
    }
  }
  return values;
}

}  // namespace

/**
 * Reading moves the odometry into the frame of the laser, which writing has
 * to undo, so that a file survives being read and written. The laser of the
 * file sits at (0.5, 0, 0.3) on the robot.
 */
TEST(Gm2dlIO, WritesTheEdgesItRead) {
  const std::string written =
      (std::filesystem::path{::testing::TempDir()} / "written.gm2dl").string();
  g2o::SparseOptimizer optimizer;
  ASSERT_TRUE(g2o::Gm2dlIO::readGm2dl(G2O_TEST_GM2DL, optimizer));

  ASSERT_TRUE(g2o::Gm2dlIO::writeGm2dl(written, optimizer));

  constexpr double Tolerance = 1e-12;
  const std::vector<double> expected = edges(G2O_TEST_GM2DL);
  ASSERT_THAT(expected, Not(IsEmpty()));
  EXPECT_THAT(edges(written), Pointwise(DoubleNear(Tolerance), expected));
}
