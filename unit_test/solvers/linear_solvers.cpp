// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>

#include <Eigen/Core>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "g2o/core/sparse_block_matrix.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"
#include "gtest/gtest.h"
#ifdef G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#endif
#ifdef G2O_HAVE_CSPARSE
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#endif

namespace {

using Matrix = Eigen::MatrixXd;

/**
 * Symmetric matrix of two rows in the upper triangular block storage of the
 * solvers, either as two scalar blocks or as one block.
 */
g2o::SparseBlockMatrix<Matrix> symmetric(const Eigen::Matrix2d& values,
                                         bool oneBlock) {
  if (oneBlock) {
    const std::vector<int> blocks{2};
    g2o::SparseBlockMatrix<Matrix> matrix(blocks.data(), blocks.data(), 1, 1);
    *matrix.block(0, 0, true) = values;
    return matrix;
  }
  const std::vector<int> blocks{1, 2};
  g2o::SparseBlockMatrix<Matrix> matrix(blocks.data(), blocks.data(), 2, 2);
  (*matrix.block(0, 0, true))(0, 0) = values(0, 0);
  (*matrix.block(1, 1, true))(0, 0) = values(1, 1);
  if (values(0, 1) != 0.) {
    (*matrix.block(0, 1, true))(0, 0) = values(0, 1);
  }
  return matrix;
}

template <typename Solver>
class LinearSolvers : public ::testing::Test {
 protected:
  bool solve(const g2o::SparseBlockMatrix<Matrix>& matrix,
             Eigen::Vector2d b = Eigen::Vector2d(1., 1.)) {
    Solver solver;
    EXPECT_TRUE(solver.init());
    Eigen::Vector2d x = Eigen::Vector2d::Zero();
    return solver.solve(matrix, x.data(), b.data());
  }
};

using SolverTypes = ::testing::Types<
#ifdef G2O_HAVE_CHOLMOD
    g2o::LinearSolverCholmod<Matrix>,
#endif
#ifdef G2O_HAVE_CSPARSE
    g2o::LinearSolverCSparse<Matrix>,
#endif
    g2o::LinearSolverDense<Matrix>, g2o::LinearSolverEigen<Matrix>,
    g2o::LinearSolverPCG<Matrix>>;
TYPED_TEST_SUITE(LinearSolvers, SolverTypes);

}  // namespace

TYPED_TEST(LinearSolvers, SolvesAPositiveDefiniteSystem) {
  EXPECT_TRUE(this->solve(
      symmetric((Eigen::Matrix2d() << 2., 0., 0., 3.).finished(), false)));
}

//! A NaN in the system has no solution, which the step must not apply.
TYPED_TEST(LinearSolvers, RejectsANaN) {
  constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(this->solve(
      symmetric((Eigen::Matrix2d() << NaN, 0., 0., 1.).finished(), false)));
}

/**
 * The Hessian of a least squares problem is positive semidefinite, so a
 * negative curvature reveals a broken system.
 */
TYPED_TEST(LinearSolvers, RejectsAnIndefiniteSystem) {
  EXPECT_FALSE(this->solve(
      symmetric((Eigen::Matrix2d() << 1., 0., 0., -1.).finished(), false)));
}

/**
 * Positive diagonal blocks do not make the system positive definite, whose
 * coupling can still turn it indefinite.
 */
TYPED_TEST(LinearSolvers, RejectsAnIndefiniteCoupling) {
  // The right-hand side is no eigenvector, so iterations meet both
  // eigenvalues.
  EXPECT_FALSE(this->solve(
      symmetric((Eigen::Matrix2d() << 1., 2., 2., 1.).finished(), false),
      Eigen::Vector2d(1., 0.)));
}

//! A singular system, such as an unconstrained gauge, has no unique step.
TYPED_TEST(LinearSolvers, RejectsASingularSystem) {
  EXPECT_FALSE(this->solve(
      symmetric((Eigen::Matrix2d() << 1., 1., 1., 1.).finished(), true)));
}

/**
 * A failed factorization leaves the working directory as it was, which a
 * debug file of the Hessian would change behind the back of the program.
 */
TYPED_TEST(LinearSolvers, WritesNoFileOnFailure) {
  // The tests of the solvers may run at the same time.
  const std::string suite = ::testing::UnitTest::GetInstance()
                                ->current_test_info()
                                ->test_suite_name();
  const std::filesystem::path directory =
      std::filesystem::path{::testing::TempDir()} /
      ("linear_solver_failure_" + suite.substr(suite.rfind('/') + 1));
  std::filesystem::create_directories(directory);
  const std::filesystem::path previous = std::filesystem::current_path();
  std::filesystem::current_path(directory);

  this->solve(
      symmetric((Eigen::Matrix2d() << 1., 1., 1., 1.).finished(), true));

  std::filesystem::current_path(previous);
  EXPECT_TRUE(std::filesystem::is_empty(directory));
  std::filesystem::remove_all(directory);
}
