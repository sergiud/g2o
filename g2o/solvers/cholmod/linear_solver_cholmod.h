// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINEAR_SOLVER_CHOLMOD
#define G2O_LINEAR_SOLVER_CHOLMOD

#include <cholmod.h>
#include <g2o/config.h>
#include <g2o/core/batch_stats.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_block_matrix_ccs.h>
#include <g2o/prefix.h>
#include <g2o/solvers/cholmod/g2o_solver_cholmod_api.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/sparse_helper.h>
#include <g2o/stuff/timeutil.h>

#include <cstddef>
#include <memory>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief Our extension of the CHOLMOD matrix struct
 */
struct G2O_SOLVER_CHOLMOD_API CholmodExt : public cholmod_sparse {
  CholmodExt();

  //! allocates the column pointers for the number of columns
  void allocateColumns(std::size_t columns);
  //! allocates the row indices and values for the number of entries
  void allocateEntries(std::size_t entries);

  size_t columnsAllocated;

 private:
  std::unique_ptr<int[]> _columnPointers;
  std::unique_ptr<int[]> _rowIndices;
  std::unique_ptr<double[]> _values;
};

//! Frees the objects that CHOLMOD allocates with the common it used.
struct CholmodFree {
  cholmod_common* common;

  void operator()(cholmod_factor* factor) const {
    cholmod_free_factor(&factor, common);
  }
  void operator()(cholmod_dense* dense) const {
    cholmod_free_dense(&dense, common);
  }
  void operator()(cholmod_sparse* sparse) const {
    cholmod_free_sparse(&sparse, common);
  }
  void operator()(cholmod_triplet* triplet) const {
    cholmod_free_triplet(&triplet, common);
  }
};

/**
 * \brief basic solver for Ax = b which has to reimplemented for different
 * linear algebra libraries
 */
template <typename MatrixType>
class LinearSolverCholmod : public LinearSolverCCS<MatrixType> {
 public:
  LinearSolverCholmod() : LinearSolverCCS<MatrixType>() {
    _writeDebug = false;
    _blockOrdering = false;
    _cholmodSparse = std::make_unique<CholmodExt>();
    cholmod_start(&_cholmodCommon);

    // setup ordering strategy
    _cholmodCommon.nmethods = 1;
    _cholmodCommon.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
    //_cholmodCommon.postorder = 0;

    _cholmodCommon.supernodal =
        CHOLMOD_AUTO;  // CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
    // A simplicial LDL' factorization accepts negative pivots, which the
    // conversion into LL' rejects as not positive definite.
    _cholmodCommon.final_ll = 1;
  }

  virtual ~LinearSolverCholmod() {
    // The factor needs the common, which cholmod_finish releases.
    _cholmodFactor.reset();
    cholmod_finish(&_cholmodCommon);
  }

  virtual bool init() {
    _cholmodFactor.reset();
    return true;
  }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    // cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
    fillCholmodExt(
        A, _cholmodFactor !=
               nullptr);  // _cholmodFactor used as bool, if not existing will
                          // copy the whole structure, otherwise only the values

    if (_cholmodFactor == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_cholmodFactor != 0 && "Symbolic cholesky failed");
    }
    double t = get_monotonic_time();

    // setting up b for calling cholmod
    cholmod_dense bcholmod{};
    bcholmod.nrow = bcholmod.d = _cholmodSparse->nrow;
    bcholmod.ncol = 1;
    bcholmod.x = b;
    bcholmod.xtype = CHOLMOD_REAL;
    bcholmod.dtype = CHOLMOD_DOUBLE;

    cholmod_factorize(_cholmodSparse.get(), _cholmodFactor.get(),
                      &_cholmodCommon);
    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
      ngx::Log(ngx::Error, "Cholesky failure");
      if (_writeDebug) {
        ngx::Log("writing the Hessian to debug.txt, loadable by Octave");
        saveMatrix("debug.txt");
      }
      return false;
    }

    std::unique_ptr<cholmod_dense, CholmodFree> xcholmod(
        cholmod_solve(CHOLMOD_A, _cholmodFactor.get(), &bcholmod,
                      &_cholmodCommon),
        CholmodFree{&_cholmodCommon});
    memcpy(x, xcholmod->x,
           sizeof(double) * bcholmod.nrow);  // copy back to our array
    if (!VectorXD::ConstMapType(x, bcholmod.nrow).allFinite()) {
      return false;
    }

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          static_cast<size_t>(_cholmodCommon.method[0].lnz);
    }

    return true;
  }

  bool solveBlocks(std::vector<std::vector<double>>& blocks,
                   const SparseBlockMatrix<MatrixType>& A) {
    // cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
    fillCholmodExt(
        A, _cholmodFactor !=
               nullptr);  // _cholmodFactor used as bool, if not existing will
                          // copy the whole structure, otherwise only the values

    if (_cholmodFactor == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_cholmodFactor && "Symbolic cholesky failed");
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

    cholmod_factorize(_cholmodSparse.get(), _cholmodFactor.get(),
                      &_cholmodCommon);
    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) return false;

    // convert the factorization to LL, simplical, packed, monotonic
    int change_status = cholmod_change_factor(
        CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor.get(), &_cholmodCommon);
    if (change_status == 0) {
      return false;
    }
    assert(_cholmodFactor->is_ll && !_cholmodFactor->is_super &&
           _cholmodFactor->is_monotonic && "Cholesky factor has wrong format");

    // invert the permutation
    int* p = (int*)_cholmodFactor->Perm;
    VectorXI pinv;
    pinv.resize(_cholmodSparse->ncol);
    for (size_t i = 0; i < _cholmodSparse->ncol; ++i) pinv(p[i]) = i;

    // compute the marginal covariance
    MarginalCovarianceCholesky mcc;
    mcc.setCholeskyFactor(_cholmodSparse->ncol, (int*)_cholmodFactor->p,
                          (int*)_cholmodFactor->i, (double*)_cholmodFactor->x,
                          pinv.data());
    mcc.computeCovariance(blockData.data(), A.rowBlockIndices());

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->choleskyNNZ = static_cast<size_t>(
          _cholmodCommon.method[_cholmodCommon.selected].lnz);
    }

    return true;
  }

  virtual bool solvePattern(
      SparseBlockMatrix<MatrixXD>& spinv,
      const std::vector<std::pair<int, int>>& blockIndices,
      const SparseBlockMatrix<MatrixType>& A) {
    // cerr << __PRETTY_FUNCTION__ << " using cholmod" << endl;
    fillCholmodExt(
        A, _cholmodFactor !=
               nullptr);  // _cholmodFactor used as bool, if not existing will
                          // copy the whole structure, otherwise only the values

    if (_cholmodFactor == nullptr) {
      computeSymbolicDecomposition(A);
      assert(_cholmodFactor && "Symbolic cholesky failed");
    }

    cholmod_factorize(_cholmodSparse.get(), _cholmodFactor.get(),
                      &_cholmodCommon);
    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) return false;

    // convert the factorization to LL, simplical, packed, monotonic
    int change_status = cholmod_change_factor(
        CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor.get(), &_cholmodCommon);
    if (change_status == 0) {
      return false;
    }
    assert(_cholmodFactor->is_ll && !_cholmodFactor->is_super &&
           _cholmodFactor->is_monotonic && "Cholesky factor has wrong format");

    // invert the permutation
    int* p = (int*)_cholmodFactor->Perm;
    VectorXI pinv;
    pinv.resize(_cholmodSparse->ncol);
    for (size_t i = 0; i < _cholmodSparse->ncol; ++i) pinv(p[i]) = i;

    // compute the marginal covariance
    MarginalCovarianceCholesky mcc;
    mcc.setCholeskyFactor(_cholmodSparse->ncol, (int*)_cholmodFactor->p,
                          (int*)_cholmodFactor->i, (double*)_cholmodFactor->x,
                          pinv.data());
    mcc.computeCovariance(spinv, A.rowBlockIndices(), blockIndices);

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->choleskyNNZ = static_cast<size_t>(
          _cholmodCommon.method[_cholmodCommon.selected].lnz);
    }

    return true;
  }

  //! do the AMD ordering on the blocks or on the scalar matrix
  bool blockOrdering() const { return _blockOrdering; }
  void setBlockOrdering(bool blockOrdering) { _blockOrdering = blockOrdering; }

  //! write a debug dump of the system matrix if it is not SPD in solve
  virtual bool writeDebug() const { return _writeDebug; }
  virtual void setWriteDebug(bool b) { _writeDebug = b; }

  virtual bool saveMatrix(const std::string& fileName) {
    writeCCSMatrix(fileName, _cholmodSparse->nrow, _cholmodSparse->ncol,
                   (int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
                   (double*)_cholmodSparse->x, true);
    return true;
  }

 protected:
  // temp used for cholesky with cholmod
  cholmod_common _cholmodCommon{};
  std::unique_ptr<CholmodExt> _cholmodSparse;
  std::unique_ptr<cholmod_factor, CholmodFree> _cholmodFactor{
      nullptr, CholmodFree{&_cholmodCommon}};
  bool _blockOrdering;
  MatrixStructure _matrixStructure;
  VectorXI _scalarPermutation, _blockPermutation;
  bool _writeDebug;

  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    double t = get_monotonic_time();
    if (!_blockOrdering) {
      // setup ordering strategy
      _cholmodCommon.nmethods = 1;
      _cholmodCommon.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
      _cholmodFactor.reset(cholmod_analyze(
          _cholmodSparse.get(), &_cholmodCommon));  // symbolic factorization
    } else {
      A.fillBlockStructure(_matrixStructure);

      // get the ordering for the block matrix
      if (_blockPermutation.size() == 0)
        _blockPermutation.resize(_matrixStructure.n);
      if (_blockPermutation.size() <
          _matrixStructure.n)  // double space if resizing
        _blockPermutation.resize(2 * _matrixStructure.n);

      // prepare AMD call via CHOLMOD
      cholmod_sparse auxCholmodSparse{};
      auxCholmodSparse.nzmax = _matrixStructure.nzMax();
      auxCholmodSparse.nrow = auxCholmodSparse.ncol = _matrixStructure.n;
      auxCholmodSparse.p = _matrixStructure.Ap.get();
      auxCholmodSparse.i = _matrixStructure.Aii.get();
      auxCholmodSparse.nz = nullptr;
      auxCholmodSparse.x = nullptr;
      auxCholmodSparse.z = nullptr;
      auxCholmodSparse.stype = 1;
      auxCholmodSparse.xtype = CHOLMOD_PATTERN;
      auxCholmodSparse.itype = CHOLMOD_INT;
      auxCholmodSparse.dtype = CHOLMOD_DOUBLE;
      auxCholmodSparse.sorted = 1;
      auxCholmodSparse.packed = 1;
      int amdStatus = cholmod_amd(&auxCholmodSparse, nullptr, 0,
                                  _blockPermutation.data(), &_cholmodCommon);
      if (amdStatus == 0) {
        return;
      }

      // blow up the permutation to the scalar matrix
      if (_scalarPermutation.size() == 0)
        _scalarPermutation.resize(_cholmodSparse->ncol);
      if (_scalarPermutation.size() < (int)_cholmodSparse->ncol)
        _scalarPermutation.resize(2 * _cholmodSparse->ncol);
      size_t scalarIdx = 0;
      for (int i = 0; i < _matrixStructure.n; ++i) {
        const int& p = _blockPermutation(i);
        int base = A.colBaseOfBlock(p);
        int nCols = A.colsOfBlock(p);
        for (int j = 0; j < nCols; ++j)
          _scalarPermutation(scalarIdx++) = base++;
      }
      assert(scalarIdx == _cholmodSparse->ncol);

      // apply the ordering
      _cholmodCommon.nmethods = 1;
      _cholmodCommon.method[0].ordering = CHOLMOD_GIVEN;
      _cholmodFactor.reset(cholmod_analyze_p(_cholmodSparse.get(),
                                             _scalarPermutation.data(), nullptr,
                                             0, &_cholmodCommon));
    }
    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr)
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;

    // const int& bestIdx = _cholmodCommon.selected;
    // cerr << "# Number of nonzeros in L: " <<
    // (int)_cholmodCommon.method[bestIdx].lnz << " by "
    //<< (_cholmodCommon.method[bestIdx].ordering == CHOLMOD_GIVEN ? "block" :
    //"scalar") << " AMD ordering " << endl;
  }

  void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    if (!onlyValues) this->initMatrixStructure(A);
    size_t m = A.rows();
    size_t n = A.cols();
    assert(m > 0 && n > 0 && "Hessian has 0 rows/cols");

    if (_cholmodSparse->columnsAllocated < n) {
      // std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" <<
      // std::endl;
      _cholmodSparse->columnsAllocated =
          _cholmodSparse->columnsAllocated == 0
              ? n
              : 2 * n;  // pre-allocate more space if re-allocating
      _cholmodSparse->allocateColumns(_cholmodSparse->columnsAllocated);
    }
    if (!onlyValues) {
      size_t nzmax = A.nonZeros();
      if (_cholmodSparse->nzmax < nzmax) {
        // std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" <<
        // std::endl;
        _cholmodSparse->nzmax =
            _cholmodSparse->nzmax == 0
                ? nzmax
                : 2 * nzmax;  // pre-allocate more space if re-allocating
        _cholmodSparse->allocateEntries(_cholmodSparse->nzmax);
      }
    }
    _cholmodSparse->ncol = n;
    _cholmodSparse->nrow = m;

    if (onlyValues)
      this->_ccsMatrix->fillCCS((double*)_cholmodSparse->x, true);
    else
      this->_ccsMatrix->fillCCS((int*)_cholmodSparse->p,
                                (int*)_cholmodSparse->i,
                                (double*)_cholmodSparse->x, true);
  }
};

G2O_EXTERN_TEMPLATE class G2O_SOLVER_CHOLMOD_API
    LinearSolverCholmod<Eigen::MatrixXd>;

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
