// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/core/sparse_block_matrix.h"

#include <Eigen/Core>
#include <memory>
#include <vector>

#include "gtest/gtest.h"

//! The product of two matrices of matching blocks succeeds.
TEST(SparseBlockMatrix, ReportsAProduct) {
  const std::vector<int> blocks{2, 4};
  g2o::SparseBlockMatrix<Eigen::MatrixXd> left(blocks.data(), blocks.data(), 2,
                                               2);
  g2o::SparseBlockMatrix<Eigen::MatrixXd> right(blocks.data(), blocks.data(), 2,
                                                2);
  *left.block(0, 0, true) = Eigen::Matrix2d::Identity();
  *right.block(0, 1, true) = 2. * Eigen::Matrix2d::Identity();
  std::unique_ptr<g2o::SparseBlockMatrix<Eigen::MatrixXd>> product;

  EXPECT_TRUE(left.multiply(product, &right));

  ASSERT_NE(product, nullptr);
  ASSERT_NE(product->block(0, 1), nullptr);
  EXPECT_EQ(*product->block(0, 1), 2. * Eigen::Matrix2d::Identity());
}

//! Permuting a matrix without blocks yields another matrix without blocks.
TEST(SparseBlockMatrix, PermutesAnEmptyMatrix) {
  const g2o::SparseBlockMatrix<Eigen::MatrixXd> empty;
  std::unique_ptr<g2o::SparseBlockMatrix<Eigen::MatrixXd>> permuted;

  EXPECT_TRUE(empty.symmPermutation(permuted, nullptr, true));

  ASSERT_NE(permuted, nullptr);
  EXPECT_EQ(permuted->rows(), 0);
}

/**
 * Swapping two blocks of an upper triangular matrix moves their diagonal
 * blocks and keeps the coupling in the upper triangle, transposed.
 */
TEST(SparseBlockMatrix, PermutesTheBlocksOfAnUpperTriangle) {
  const std::vector<int> blocks{2, 5};
  g2o::SparseBlockMatrix<Eigen::MatrixXd> matrix(blocks.data(), blocks.data(),
                                                 2, 2);
  *matrix.block(0, 0, true) = Eigen::Matrix2d::Identity();
  *matrix.block(1, 1, true) = 2. * Eigen::Matrix3d::Identity();
  const Eigen::Matrix<double, 2, 3> coupling{{1., 2., 3.}, {4., 5., 6.}};
  *matrix.block(0, 1, true) = coupling;
  const std::vector<int> swap{1, 0};
  std::unique_ptr<g2o::SparseBlockMatrix<Eigen::MatrixXd>> permuted;

  ASSERT_TRUE(matrix.symmPermutation(permuted, swap.data(), true));

  ASSERT_NE(permuted, nullptr);
  const g2o::SparseBlockMatrix<Eigen::MatrixXd>& result = *permuted;
  ASSERT_NE(result.block(0, 0), nullptr);
  ASSERT_NE(result.block(0, 1), nullptr);
  ASSERT_NE(result.block(1, 1), nullptr);
  EXPECT_EQ(*result.block(0, 0), 2. * Eigen::Matrix3d::Identity());
  EXPECT_EQ(*result.block(1, 1), Eigen::Matrix2d::Identity());
  EXPECT_EQ(*result.block(0, 1), coupling.transpose());
  EXPECT_EQ(result.block(1, 0), nullptr);
}
