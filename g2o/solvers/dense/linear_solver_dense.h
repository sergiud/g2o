// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2012 R. Kümmerle
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINEAR_SOLVER_DENSE_H
#define G2O_LINEAR_SOLVER_DENSE_H

#include <g2o/core/batch_stats.h>
#include <g2o/core/linear_solver.h>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <utility>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief linear solver using dense cholesky decomposition
 */
template <typename MatrixType>
class LinearSolverDense : public LinearSolver<MatrixType> {
 public:
  LinearSolverDense() : LinearSolver<MatrixType>(), _reset(true) {}

  virtual ~LinearSolverDense() = default;

  virtual bool init() {
    _reset = true;
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    int n = A.cols();
    int m = A.cols();

    MatrixXD& H = _H;
    if (H.cols() != n) {
      H.resize(n, m);
      _reset = true;
    }
    if (_reset) {
      _reset = false;
      H.setZero();
    }

    // copy the sparse block matrix into a dense matrix
    int c_idx = 0;
    for (size_t i = 0; i < A.blockCols().size(); ++i) {
      int c_size = A.colsOfBlock(i);
      assert(c_idx == A.colBaseOfBlock(i) && "mismatch in block indices");

      const typename SparseBlockMatrix<MatrixType>::IntBlockMap& col =
          A.blockCols()[i];
      if (!col.empty()) {
        typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator it;
        for (it = col.begin(); it != col.end(); ++it) {
          int r_idx = A.rowBaseOfBlock(it->first);
          // only the upper triangular block is processed
          if (it->first <= (int)i) {
            int r_size = A.rowsOfBlock(it->first);
            H.block(r_idx, c_idx, r_size, c_size) = *(it->second);
            if (r_idx != c_idx)  // write the lower triangular block
              H.block(c_idx, r_idx, c_size, r_size) = it->second->transpose();
          }
        }
      }

      c_idx += c_size;
    }

    // solving via Cholesky decomposition
    VectorXD::MapType xvec(x, m);
    VectorXD::ConstMapType bvec(b, n);
    _cholesky.compute(H);
    if (_cholesky.info() != Eigen::Success) {
      return false;
    }
    xvec = _cholesky.solve(bvec);
    return xvec.allFinite();
  }

 protected:
  bool _reset;
  MatrixXD _H;
  Eigen::LLT<MatrixXD> _cholesky;
};

G2O_END_NAMESPACE

#endif
