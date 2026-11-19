// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/solvers/csparse/csparse_helper.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::ElementsAre;

namespace {

/**
 * The upper triangle of the symmetric matrix
 * @f$\begin{bmatrix} 1 & 2 \\ 2 & 3 \end{bmatrix}@f$ as the entries of an
 * Octave file, sorted by column.
 */
constexpr auto Expected = {"1 1 1", "2 1 2", "1 2 2", "2 2 3"};

//! Writes a matrix and returns the lines of its entries.
std::vector<std::string> write(const cs* matrix) {
  const std::filesystem::path file =
      std::filesystem::path(::testing::TempDir()) / "csparse_helper.txt";
  EXPECT_TRUE(g2o::csparse_extension::writeCs2Octave(file.string().c_str(),
                                                     matrix,
                                                     /*upperTriangular=*/true));
  std::ifstream is(file);
  std::vector<std::string> entries;
  for (std::string line; std::getline(is, line);) {
    if (!line.empty() && line.front() != '#') {
      entries.push_back(line);
    }
  }
  std::filesystem::remove(file);
  return entries;
}

}  // namespace

TEST(CSparseHelper, WritesACompressedMatrix) {
  std::vector<std::int32_t> columns{0, 1, 3};
  std::vector<std::int32_t> rows{0, 0, 1};
  std::vector<double> values{1., 2., 3.};
  const cs matrix{3, 2, 2, columns.data(), rows.data(), values.data(), -1};

  EXPECT_THAT(write(&matrix),
              ElementsAre(Expected.begin()[0], Expected.begin()[1],
                          Expected.begin()[2], Expected.begin()[3]));
}

TEST(CSparseHelper, WritesTriplets) {
  std::vector<std::int32_t> columns{1, 0, 1};
  std::vector<std::int32_t> rows{1, 0, 0};
  std::vector<double> values{3., 1., 2.};
  const cs matrix{3, 2, 2, columns.data(), rows.data(), values.data(), 3};

  EXPECT_THAT(write(&matrix),
              ElementsAre(Expected.begin()[0], Expected.begin()[1],
                          Expected.begin()[2], Expected.begin()[3]));
}

/**
 * A failed factorization is the caller's to report, through the log, rather
 * than a message the helper writes past every log sink.
 */
TEST(CSparseHelper, LeavesTheReportOfAFailedFactorizationToItsCaller) {
  constexpr int Size = 2;
  // The upper triangle of the indefinite matrix [[1, 2], [2, 1]].
  std::vector<std::int32_t> columns{0, 1, 3};
  std::vector<std::int32_t> rows{0, 0, 1};
  std::vector<double> values{1., 2., 1.};
  const cs matrix{3,           Size,          Size, columns.data(),
                  rows.data(), values.data(), -1};
  const std::unique_ptr<css, decltype(&cs_sfree)> symbolic(
      cs_schol(/*order=*/0, &matrix), &cs_sfree);
  ASSERT_NE(symbolic, nullptr);
  std::vector<double> b{1., 1.};
  std::vector<double> workspace(2 * Size);
  std::vector<int> intWorkspace(4 * Size);

  ::testing::internal::CaptureStderr();
  const int ok = g2o::csparse_extension::cs_cholsolsymb(
      &matrix, b.data(), symbolic.get(), workspace.data(), intWorkspace.data());
  const std::string output = ::testing::internal::GetCapturedStderr();

  EXPECT_EQ(ok, 0);
  EXPECT_THAT(output, ::testing::IsEmpty());
}
