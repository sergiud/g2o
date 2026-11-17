// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINEAR_SOLVER_PCG_H
#define G2O_LINEAR_SOLVER_PCG_H

#include <g2o/core/batch_stats.h>
#include <g2o/core/linear_solver.h>
#include <g2o/stuff/logger.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <utility>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief linear solver using PCG, pre-conditioner is block Jacobi
 */
template <typename MatrixType>
class LinearSolverPCG : public LinearSolver<MatrixType> {
 public:
  LinearSolverPCG() : LinearSolver<MatrixType>() {
    _tolerance = 1e-6;
    _verbose = false;
    _absoluteTolerance = true;
    _residual = -1.0;
    _maxIter = -1;
  }

  virtual ~LinearSolverPCG() = default;

  virtual bool init() {
    _residual = -1.0;
    _indices.clear();
    _sparseMat.clear();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b);

  //! return the tolerance for terminating PCG before convergence
  double tolerance() const { return _tolerance; }
  void setTolerance(double tolerance) { _tolerance = tolerance; }

  int maxIterations() const { return _maxIter; }
  void setMaxIterations(int maxIter) { _maxIter = maxIter; }

  bool absoluteTolerance() const { return _absoluteTolerance; }
  void setAbsoluteTolerance(bool absoluteTolerance) {
    _absoluteTolerance = absoluteTolerance;
  }

  bool verbose() const { return _verbose; }
  void setVerbose(bool verbose) { _verbose = verbose; }

 protected:
  typedef std::vector<MatrixType, Eigen::aligned_allocator<MatrixType> >
      MatrixVector;
  typedef std::vector<const MatrixType*> MatrixPtrVector;

  double _tolerance;
  double _residual;
  bool _absoluteTolerance;
  bool _verbose;
  int _maxIter;

  MatrixPtrVector _diag;
  MatrixVector _J;

  std::vector<std::pair<int, int> > _indices;
  MatrixPtrVector _sparseMat;

  void multDiag(const std::vector<int>& colBlockIndices, MatrixVector& A,
                const VectorXD& src, VectorXD& dest);
  void multDiag(const std::vector<int>& colBlockIndices, MatrixPtrVector& A,
                const VectorXD& src, VectorXD& dest);
  void mult(const std::vector<int>& colBlockIndices, const VectorXD& src,
            VectorXD& dest);
};

#include "linear_solver_pcg.hpp"

G2O_END_NAMESPACE

#endif
