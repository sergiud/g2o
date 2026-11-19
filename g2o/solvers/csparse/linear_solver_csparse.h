// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2015-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINEAR_SOLVERCSPARSE_H
#define G2O_LINEAR_SOLVERCSPARSE_H

#include <g2o/config.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/marginal_covariance_cholesky.h>
#include <g2o/prefix.h>
#include <g2o/solvers/csparse/csparse_helper.h>
#include <g2o/solvers/csparse/g2o_solver_csparse_api.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/timeutil.h>

#include <iostream>
#include <memory>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief Our C++ version of the csparse struct
 */
struct G2O_SOLVER_CSPARSE_API CSparseExt : public cs {
  CSparseExt() : cs{} {}

  //! allocates the column pointers for the number of columns
  void allocateColumns(int columns) {
    _columnPointers = std::make_unique<int[]>(columns + 1);
    p = _columnPointers.get();
  }

  //! allocates the row indices and values for the number of entries
  void allocateEntries(int entries) {
    _rowIndices = std::make_unique<int[]>(entries);
    _values = std::make_unique<double[]>(entries);
    i = _rowIndices.get();
    x = _values.get();
  }

  int columnsAllocated{};

  CSparseExt(const CSparseExt&) = delete;
  CSparseExt& operator=(const CSparseExt&) = delete;

 private:
  std::unique_ptr<int[]> _columnPointers;
  std::unique_ptr<int[]> _rowIndices;
  std::unique_ptr<double[]> _values;
};

//! Frees the objects that CSparse allocates.
struct CSparseFree {
  void operator()(css* symbolic) const { cs_sfree(symbolic); }
  void operator()(csn* numeric) const { cs_nfree(numeric); }
  void operator()(cs* matrix) const { cs_spfree(matrix); }
  void operator()(int* array) const { cs_free(array); }
};

/**
 * \brief linear solver which uses CSparse
 */
template <typename MatrixType>
class LinearSolverCSparse : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCSparse() : LinearSolverCCS<MatrixType>() {
    _csWorkspaceSize = -1;
    _ccsA = std::make_unique<CSparseExt>();
    _blockOrdering = true;
    _writeDebug = false;
  }

  virtual ~LinearSolverCSparse() = default;

  virtual bool init() {
    _symbolicDecomposition.reset();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    fillCSparse(A, _symbolicDecomposition != nullptr);
    // perform symbolic cholesky once
    if (_symbolicDecomposition == nullptr) {
      computeSymbolicDecomposition(A);
    }
    // re-allocate the temporary workspace for cholesky
    if (_csWorkspaceSize < _ccsA->n) {
      _csWorkspaceSize = 2 * _ccsA->n;
      _csWorkspace = std::make_unique<double[]>(_csWorkspaceSize);
      _csIntWorkspace = std::make_unique<int[]>(2 * _csWorkspaceSize);
    }

    double t = get_monotonic_time();
    // _x = _b for calling csparse
    if (x != b) memcpy(x, b, _ccsA->n * sizeof(double));
    int ok = csparse_extension::cs_cholsolsymb(
        _ccsA.get(), x, _symbolicDecomposition.get(), _csWorkspace.get(),
        _csIntWorkspace.get());
    if (ok == 0) {
      ngx::Log(ngx::Error, "Cholesky failure");
      if (_writeDebug) {
        ngx::Log("writing the Hessian to debug.txt, loadable by Octave");
        csparse_extension::writeCs2Octave("debug.txt", _ccsA.get(), true);
      }
      return false;
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          static_cast<size_t>(_symbolicDecomposition->lnz);
    }

    return ok != 0;
  }

  bool solveBlocks(std::vector<std::vector<double>>& blocks,
                   const SparseBlockMatrix<MatrixType>& A) {
    fillCSparse(A, _symbolicDecomposition != nullptr);
    // perform symbolic cholesky once
    if (_symbolicDecomposition == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_symbolicDecomposition && "Symbolic cholesky failed");
    }
    // re-allocate the temporary workspace for cholesky
    if (_csWorkspaceSize < _ccsA->n) {
      _csWorkspaceSize = 2 * _ccsA->n;
      _csWorkspace = std::make_unique<double[]>(_csWorkspaceSize);
      _csIntWorkspace = std::make_unique<int[]>(2 * _csWorkspaceSize);
    }

    if (blocks.empty()) {
      blocks.resize(A.rowBlockIndices().size());
      for (size_t i = 0; i < A.rowBlockIndices().size(); ++i) {
        blocks[i].resize(A.rowsOfBlock(i) * A.colsOfBlock(i));
      }
    }
    std::vector<double*> blockData;
    blockData.reserve(blocks.size());
    for (auto& block : blocks) {
      blockData.push_back(block.data());
    }

    int ok = 1;
    std::unique_ptr<csn, CSparseFree> numericCholesky(
        csparse_extension::cs_chol_workspace(
            _ccsA.get(), _symbolicDecomposition.get(), _csIntWorkspace.get(),
            _csWorkspace.get()));
    if (numericCholesky != nullptr) {
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p,
                            numericCholesky->L->i, numericCholesky->L->x,
                            _symbolicDecomposition->pinv);
      mcc.computeCovariance(blockData.data(), A.rowBlockIndices());
    } else {
      ok = 0;
      ngx::Log(ngx::Error, "inverse fail (numeric decomposition)");
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->choleskyNNZ =
          static_cast<size_t>(_symbolicDecomposition->lnz);
    }

    return ok != 0;
  }

  virtual bool solvePattern(
      SparseBlockMatrix<MatrixXD>& spinv,
      const std::vector<std::pair<int, int>>& blockIndices,
      const SparseBlockMatrix<MatrixType>& A) {
    fillCSparse(A, _symbolicDecomposition != nullptr);
    // perform symbolic cholesky once
    if (_symbolicDecomposition == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_symbolicDecomposition && "Symbolic cholesky failed");
    }
    // re-allocate the temporary workspace for cholesky
    if (_csWorkspaceSize < _ccsA->n) {
      _csWorkspaceSize = 2 * _ccsA->n;
      _csWorkspace = std::make_unique<double[]>(_csWorkspaceSize);
      _csIntWorkspace = std::make_unique<int[]>(2 * _csWorkspaceSize);
    }

    int ok = 1;
    std::unique_ptr<csn, CSparseFree> numericCholesky(
        csparse_extension::cs_chol_workspace(
            _ccsA.get(), _symbolicDecomposition.get(), _csIntWorkspace.get(),
            _csWorkspace.get()));
    if (numericCholesky != nullptr) {
      MarginalCovarianceCholesky mcc;
      mcc.setCholeskyFactor(_ccsA->n, numericCholesky->L->p,
                            numericCholesky->L->i, numericCholesky->L->x,
                            _symbolicDecomposition->pinv);
      mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);
    } else {
      ok = 0;
      ngx::Log(ngx::Error, "inverse fail (numeric decomposition)");
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->choleskyNNZ =
          static_cast<size_t>(_symbolicDecomposition->lnz);
    }

    return ok != 0;
  }

  //! do the AMD ordering on the blocks or on the scalar matrix
  bool blockOrdering() const { return _blockOrdering; }
  void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering; }

  //! write a debug dump of the system matrix if it is not SPD in solve
  virtual bool writeDebug() const { return _writeDebug; }
  virtual void setWriteDebug(bool b) { _writeDebug = b; }

 protected:
  std::unique_ptr<css, CSparseFree> _symbolicDecomposition;
  int _csWorkspaceSize;
  std::unique_ptr<double[]> _csWorkspace;
  std::unique_ptr<int[]> _csIntWorkspace;
  std::unique_ptr<CSparseExt> _ccsA;
  bool _blockOrdering;
  MatrixStructure _matrixStructure;
  VectorXI _scalarPermutation;
  bool _writeDebug;

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    double t = get_monotonic_time();
    if (!_blockOrdering) {
      _symbolicDecomposition.reset(cs_schol(1, _ccsA.get()));
    } else {
      A.fillBlockStructure(_matrixStructure);

      // prepare block structure for the CSparse call
      cs auxBlock{};
      auxBlock.nzmax = _matrixStructure.nzMax();
      auxBlock.m = auxBlock.n = _matrixStructure.n;
      auxBlock.p = _matrixStructure.Ap.get();
      auxBlock.i = _matrixStructure.Aii.get();
      auxBlock.x = nullptr;  // no values
      auxBlock.nz = -1;      // CCS format

      // AMD ordering on the block structure
      const int& n = _ccsA->n;
      std::unique_ptr<int, CSparseFree> P(cs_amd(1, &auxBlock));

      // blow up the permutation to the scalar matrix
      if (_scalarPermutation.size() == 0) _scalarPermutation.resize(n);
      if (_scalarPermutation.size() < n) _scalarPermutation.resize(2 * n);
      size_t scalarIdx = 0;
      for (int i = 0; i < _matrixStructure.n; ++i) {
        const int& p = P.get()[i];
        int base = A.colBaseOfBlock(p);
        int nCols = A.colsOfBlock(p);
        for (int j = 0; j < nCols; ++j)
          _scalarPermutation(scalarIdx++) = base++;
      }
      assert((int)scalarIdx == n);

      // apply the scalar permutation to finish symbolic decomposition
      _symbolicDecomposition.reset(
          (css*)cs_calloc(1, sizeof(css))); /* allocate result S */
      _symbolicDecomposition->pinv = cs_pinv(_scalarPermutation.data(), n);
      std::unique_ptr<cs, CSparseFree> C(
          cs_symperm(_ccsA.get(), _symbolicDecomposition->pinv, 0));
      _symbolicDecomposition->parent = cs_etree(C.get(), 0);
      std::unique_ptr<int, CSparseFree> post(
          cs_post(_symbolicDecomposition->parent, n));
      std::unique_ptr<int, CSparseFree> c(
          cs_counts(C.get(), _symbolicDecomposition->parent, post.get(), 0));
      _symbolicDecomposition->cp = (int*)cs_malloc(n + 1, sizeof(int));
      _symbolicDecomposition->unz = _symbolicDecomposition->lnz =
          cs_cumsum(_symbolicDecomposition->cp, c.get(), n);
      if (_symbolicDecomposition->lnz < 0) {
        _symbolicDecomposition.reset();
      }
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
    }

    /* std::cerr << "# Number of nonzeros in L: " <<
     * (int)_symbolicDecomposition->lnz << " by " */
    /*   << (_blockOrdering ? "block" : "scalar") << " AMD ordering " <<
     * std::endl; */
  }

  void fillCSparse(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    if (!onlyValues) this->initMatrixStructure(A);
    int m = A.rows();
    int n = A.cols();
    assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

    if (_ccsA->columnsAllocated < n) {
      _ccsA->columnsAllocated =
          _ccsA->columnsAllocated == 0
              ? n
              : 2 * n;  // pre-allocate more space if re-allocating
      _ccsA->allocateColumns(_ccsA->columnsAllocated);
    }

    if (!onlyValues) {
      int nzmax = A.nonZeros();
      if (_ccsA->nzmax < nzmax) {
        _ccsA->nzmax =
            _ccsA->nzmax == 0
                ? nzmax
                : 2 * nzmax;  // pre-allocate more space if re-allocating
        _ccsA->allocateEntries(_ccsA->nzmax);
      }
    }
    _ccsA->m = m;
    _ccsA->n = n;

    if (onlyValues) {
      this->_ccsMatrix->fillCCS(_ccsA->x, true);
    } else {
      int nz = this->_ccsMatrix->fillCCS(_ccsA->p, _ccsA->i, _ccsA->x, true);
      (void)nz;
      assert(nz <= _ccsA->nzmax);
    }
    _ccsA->nz = -1;  // tag as CCS formatted matrix
  }
};

G2O_EXTERN_TEMPLATE class G2O_SOLVER_CSPARSE_API
LinearSolverCSparse<Eigen::MatrixXd>;

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
