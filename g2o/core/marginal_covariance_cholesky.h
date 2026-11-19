// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_MARGINAL_COVARIANCE_CHOLESKY_H
#define G2O_MARGINAL_COVARIANCE_CHOLESKY_H

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/prefix.h>

#include <cassert>
#include <cstdint>
#include <unordered_map>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief computing the marginal covariance given a cholesky factor (lower
 * triangle of the factor)
 */
class G2O_CORE_API MarginalCovarianceCholesky {
 protected:
  /**
   * hash struct for storing the matrix elements needed to compute the
   * covariance
   */
  typedef std::unordered_map<std::int64_t, double> LookupMap;

 public:
  MarginalCovarianceCholesky();
  ~MarginalCovarianceCholesky();

  /**
   * compute the marginal cov for the given block indices, write the result to
   * the covBlocks memory (which has to be provided by the caller).
   */
  void computeCovariance(double** covBlocks,
                         const std::vector<int>& blockIndices);

  /**
   * compute the marginal cov for the given block indices, write the result in
   * spinv).
   */
  void computeCovariance(SparseBlockMatrix<MatrixXD>& spinv,
                         const std::vector<int>& rowBlockIndices,
                         const std::vector<std::pair<int, int> >& blockIndices);

  /**
   * set the CCS representation of the cholesky factor along with the inverse
   * permutation used to reduce the fill-in. permInv might be 0, will then not
   * permute the entries.
   *
   * The pointers provided by the user need to be still valid when calling
   * computeCovariance(). The pointers are owned by the caller,
   * MarginalCovarianceCholesky does not free the pointers.
   */
  void setCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv);

 protected:
  // information about the cholesky factor (lower triangle)
  int _n;       ///< L is an n X n matrix
  int* _Ap;     ///< column pointer of the CCS storage
  int* _Ai;     ///< row indices of the CCS storage
  double* _Ax;  ///< values of the cholesky factor
  int* _perm;  ///< permutation of the cholesky factor. Variable re-ordering for
               ///< better fill-in

  LookupMap _map;  ///< hash look up table for the already computed entries
  std::vector<double> _diag;  ///< cache 1 / H_ii to avoid recalculations

  /**
   * Index of an entry for hashing, which needs the square of the dimension of
   * the factor and exceeds an int for large ones.
   */
  std::int64_t computeIndex(int r, int c) const {
    return static_cast<std::int64_t>(r) * _n + c;
  }
  /**
   * Computes one entry of the covariance, whose row and column are permuted
   * and upper triangular, together with the entries below it that it depends
   * on. A stack of pending entries replaces the recursion, whose depth would
   * grow with the dimension of the factor.
   */
  double computeEntry(int r, int c);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
