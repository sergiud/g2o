// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/matrix_structure.h"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

/**
 * Octave numbers the rows and columns of a sparse matrix from one, so the
 * structure of a single diagonal entry lists it at row and column one.
 */
TEST(MatrixStructure, WritesOneBasedIndices) {
  const std::filesystem::path file =
      std::filesystem::path(::testing::TempDir()) / "matrix_structure.txt";
  g2o::MatrixStructure structure;
  structure.alloc(1, 1);
  structure.Ap[0] = 0;
  structure.Ap[1] = 1;
  structure.Aii[0] = 0;

  ASSERT_TRUE(structure.write(file.string().c_str()));

  std::ifstream is(file);
  std::vector<std::string> entries;
  for (std::string line; std::getline(is, line);) {
    if (!line.empty() && line.front() != '#') {
      entries.push_back(line);
    }
  }
  std::filesystem::remove(file);
  EXPECT_THAT(entries, ElementsAre("1 1 0"));
}
