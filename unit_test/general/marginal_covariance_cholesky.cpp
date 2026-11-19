// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/marginal_covariance_cholesky.h"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/eigen_distance.h"

using ::testing::DoubleEq;
using ::testing::DoubleNear;

/**
 * The entries of a large factor need indices beyond those of an int, since
 * their count is the square of its dimension.
 */
TEST(MarginalCovarianceCholesky, InvertsALargeDiagonalFactor) {
  // The square of the dimension plus one is 2^34, whose int collides.
  constexpr int Dimension = 131071;
  std::vector<int> columns(Dimension + 1);
  std::vector<int> rows(Dimension);
  std::vector<double> values(Dimension);
  for (int i = 0; i < Dimension; ++i) {
    columns[i] = i;
    rows[i] = i;
    values[i] = i + 1.;
  }
  columns[Dimension] = Dimension;
  g2o::MarginalCovarianceCholesky covariance;
  covariance.setCholeskyFactor(Dimension, columns.data(), rows.data(),
                               values.data(), nullptr);
  std::vector<int> blockEnds;
  std::vector<double> diagonal(Dimension);
  std::vector<double*> blocks;
  for (int i = 0; i < Dimension; ++i) {
    blockEnds.push_back(i + 1);
    blocks.push_back(&diagonal[i]);
  }

  covariance.computeCovariance(blocks.data(), blockEnds);

  std::vector<::testing::Matcher<double>> expected;
  for (int i = 0; i < Dimension; ++i) {
    expected.push_back(DoubleEq(1. / ((i + 1.) * (i + 1.))));
  }
  EXPECT_THAT(diagonal, ::testing::ElementsAreArray(expected));
}

/**
 * The covariance of the first pose of a long chain depends on every later
 * row of the factor, which a recursion per row would have to nest.
 */
TEST(MarginalCovarianceCholesky, InvertsTheFactorOfALongChain) {
  constexpr int Length = 1000000;
  constexpr double Diagonal = 2.;
  constexpr double OffDiagonal = -1.;
  std::vector<int> columns(Length + 1);
  std::vector<int> rows;
  std::vector<double> values;
  for (int i = 0; i < Length; ++i) {
    columns[i] = static_cast<int>(rows.size());
    rows.push_back(i);
    values.push_back(Diagonal);
    if (i + 1 < Length) {
      rows.push_back(i + 1);
      values.push_back(OffDiagonal);
    }
  }
  columns[Length] = static_cast<int>(rows.size());
  g2o::MarginalCovarianceCholesky covariance;
  covariance.setCholeskyFactor(Length, columns.data(), rows.data(),
                               values.data(), nullptr);
  double first = 0.;
  double* blocks[] = {&first};

  covariance.computeCovariance(blocks, {1});

  // The infinite chain converges to 1/3 geometrically by a factor of 1/4.
  EXPECT_THAT(first, DoubleNear(1. / 3., 1e-12));
}

//! The marginal covariances are the diagonal blocks of the inverse.
TEST(MarginalCovarianceCholesky, MatchesTheInverseOfADenseMatrix) {
  constexpr int Dimension = 6;
  constexpr double Tolerance = 1e-12;
  const Eigen::MatrixXd A = Eigen::MatrixXd::Random(Dimension, Dimension);
  const Eigen::MatrixXd H =
      A * A.transpose() +
      Dimension * Eigen::MatrixXd::Identity(Dimension, Dimension);
  const Eigen::MatrixXd L = H.llt().matrixL();
  // The factor in compressed column storage of its lower triangle.
  std::vector<int> columns;
  std::vector<int> rows;
  std::vector<double> values;
  for (int c = 0; c < Dimension; ++c) {
    columns.push_back(static_cast<int>(rows.size()));
    for (int r = c; r < Dimension; ++r) {
      rows.push_back(r);
      values.push_back(L(r, c));
    }
  }
  columns.push_back(static_cast<int>(rows.size()));
  g2o::MarginalCovarianceCholesky covariance;
  covariance.setCholeskyFactor(Dimension, columns.data(), rows.data(),
                               values.data(), nullptr);
  constexpr int BlockSize = 2;
  std::vector<Eigen::Matrix2d> blocks(Dimension / BlockSize);
  std::vector<double*> blockData;
  std::vector<int> blockEnds;
  for (int i = 0; i < Dimension / BlockSize; ++i) {
    blockData.push_back(blocks[i].data());
    blockEnds.push_back((i + 1) * BlockSize);
  }

  covariance.computeCovariance(blockData.data(), blockEnds);

  const Eigen::MatrixXd inverse = H.inverse();
  for (int i = 0; i < Dimension / BlockSize; ++i) {
    EXPECT_THAT(blocks[i],
                ::testing::DistanceFrom(
                    Eigen::Matrix2d(inverse.block<BlockSize, BlockSize>(
                        i * BlockSize, i * BlockSize)),
                    g2o::test::MaxAbsDifference, ::testing::Le(Tolerance)))
        << "block " << i;
  }
}
