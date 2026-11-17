// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINEAR_SOLVER_H
#define G2O_LINEAR_SOLVER_H

#include <cassert>

#include <g2o/core/marginal_covariance_cholesky.h>
#include <g2o/core/batch_stats.h>

G2O_START_NAMESPACE

/**
 * \brief basic solver for Ax = b
 *
 * basic solver for Ax = b which has to reimplemented for different linear algebra libraries.
 * A is assumed to be symmetric (only upper triangular block is stored) and positive-semi-definit.
 */
template <typename MatrixType>
class LinearSolver
{
  public:
    LinearSolver() = default;;
    virtual ~LinearSolver() = default;

    /**
     * init for operating on matrices with a different non-zero pattern like before
     */
    virtual bool init() = 0;

    /**
     * Assumes that A is the same matrix for several calls.
     * Among other assumptions, the non-zero pattern does not change!
     * If the matrix changes call init() before.
     * solve system Ax = b, x and b have to allocated beforehand!!
     */
    virtual bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) = 0;

    /**
     * Inverts the diagonal blocks of A
     * @returns false if not defined.
     */
    virtual bool solveBlocks(double**&blocks, const SparseBlockMatrix<MatrixType>& A) { (void)blocks; (void) A; return false; }


    /**
     * Inverts the a block pattern of A in spinv
     * @returns false if not defined.
     */
    virtual bool solvePattern(SparseBlockMatrix<MatrixXD>& spinv, const std::vector<std::pair<int, int> >& blockIndices, const SparseBlockMatrix<MatrixType>& A){
      (void) spinv;
      (void) blockIndices;
      (void) A;
      return false;
    }

    //! write a debug dump of the system matrix if it is not PSD in solve
    virtual bool writeDebug() const { return false;}
    virtual void setWriteDebug(bool) {}
};

/**
 * \brief Solver with faster iterating structure for the linear matrix
 */
template <typename MatrixType>
class LinearSolverCCS : public LinearSolver<MatrixType>
{
  public:
    LinearSolverCCS() : LinearSolver<MatrixType>(), _ccsMatrix(nullptr) {}
    ~LinearSolverCCS()
    {
      delete _ccsMatrix;
    }

  protected:
    SparseBlockMatrixCCS<MatrixType>* _ccsMatrix;

    void initMatrixStructure(const SparseBlockMatrix<MatrixType>& A)
    {
      delete _ccsMatrix;
      _ccsMatrix = new SparseBlockMatrixCCS<MatrixType>(A.rowBlockIndices(), A.colBlockIndices());
      A.fillSparseBlockMatrixCCS(*_ccsMatrix);
    }
};

G2O_END_NAMESPACE

#endif
