// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_LINEAR_SOLVER_CHOLMOD_ONLINE
#define G2O_LINEAR_SOLVER_CHOLMOD_ONLINE

#include <camd.h>
#include <g2o/examples/interactive_slam/g2o_incremental_api.h>
#include <g2o/prefix.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <memory>

G2O_START_NAMESPACE

/**
 * \brief generic interface for the online solver
 */
class G2O_INCREMENTAL_API LinearSolverCholmodOnlineInterface {
 public:
  LinearSolverCholmodOnlineInterface() : cmember(nullptr), batchEveryN(100) {}
  virtual int choleskyUpdate(cholmod_sparse* update) = 0;
  virtual bool solve(double* x, double* b) = 0;
  virtual cholmod_factor* L() const = 0;
  virtual size_t nonZerosInL() const = 0;
  Eigen::VectorXi* cmember;
  int batchEveryN;
};

/**
 * \brief linear solver which allows to update the cholesky factor
 */
template <typename MatrixType>
class LinearSolverCholmodOnline : public LinearSolver<MatrixType>,
                                  public LinearSolverCholmodOnlineInterface {
 public:
  LinearSolverCholmodOnline() : LinearSolver<MatrixType>() {
    _cholmodSparse = std::make_unique<CholmodExt>();
    cholmod_start(&_cholmodCommon);

    // setup ordering strategy
    _cholmodCommon.nmethods = 1;
    _cholmodCommon.method[0].ordering = CHOLMOD_AMD;  // CHOLMOD_COLAMD
    //_cholmodCommon.postorder = 0;

    _cholmodCommon.supernodal =
        CHOLMOD_AUTO;  // CHOLMOD_SUPERNODAL; //CHOLMOD_SIMPLICIAL;
    batchEveryN = 100;
  }

  virtual ~LinearSolverCholmodOnline() {
    _cholmodFactor.reset();
    cholmod_finish(&_cholmodCommon);
  }

  virtual bool init() { return true; }

  bool solve(const SparseBlockMatrix<MatrixType>& A, double* x, double* b) {
    _cholmodFactor.reset();
    fillCholmodExt(A, false);

    computeSymbolicDecomposition(A);
    assert(_cholmodFactor && "Symbolic cholesky failed");
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
      ngx::Log(ngx::Error,
               "solve(): Cholesky failure, writing debug.txt (Hessian loadable "
               "by Octave)");
      writeCCSMatrix("debug.txt", _cholmodSparse->nrow, _cholmodSparse->ncol,
                     (int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
                     (double*)_cholmodSparse->x, true);
      return false;
    }

    std::unique_ptr<cholmod_dense, CholmodFree> xcholmod(
        cholmod_solve(CHOLMOD_A, _cholmodFactor.get(), &bcholmod,
                      &_cholmodCommon),
        CholmodFree{&_cholmodCommon});
    memcpy(x, xcholmod->x,
           sizeof(double) * bcholmod.nrow);  // copy back to our array

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr) {
      globalStats->timeNumericDecomposition = get_monotonic_time() - t;
      globalStats->choleskyNNZ =
          static_cast<size_t>(_cholmodCommon.method[0].lnz);
    }

    return true;
  }

  bool blockOrdering() const { return true; }

  cholmod_factor* L() const { return _cholmodFactor.get(); }

  /**
   * return the number of non-zeros in the current factorization
   */
  size_t nonZerosInL() const {
    size_t nnz = 0;
    int* nz = (int*)_cholmodFactor->nz;
    if (nz == nullptr) return 0;
    for (size_t i = 0; i < _cholmodFactor->n; ++i) nnz += nz[i];
    return nnz;
  }

  int choleskyUpdate(cholmod_sparse* update) {
    int result =
        cholmod_updown(1, update, _cholmodFactor.get(), &_cholmodCommon);
    // std::cerr << __PRETTY_FUNCTION__ << " " << result << std::endl;
    if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
      ngx::Log(
          ngx::Error,
          "Cholesky failure, writing debug.txt (Hessian loadable by Octave)");
      writeCCSMatrix("debug.txt", _cholmodSparse->nrow, _cholmodSparse->ncol,
                     (int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
                     (double*)_cholmodSparse->x, true);
      return 0;
    }
    return result;
  }

  bool solve(double* x, double* b) {
    // setting up b for calling cholmod
    cholmod_dense bcholmod{};
    bcholmod.nrow = bcholmod.d = _cholmodSparse->nrow;
    bcholmod.ncol = 1;
    bcholmod.x = b;
    bcholmod.xtype = CHOLMOD_REAL;
    bcholmod.dtype = CHOLMOD_DOUBLE;

    std::unique_ptr<cholmod_dense, CholmodFree> xcholmod(
        cholmod_solve(CHOLMOD_A, _cholmodFactor.get(), &bcholmod,
                      &_cholmodCommon),
        CholmodFree{&_cholmodCommon});
    memcpy(x, xcholmod->x,
           sizeof(double) * bcholmod.nrow);  // copy back to our array
    return true;
  }

 protected:
  // temp used for cholesky with cholmod
  cholmod_common _cholmodCommon{};
  std::unique_ptr<CholmodExt> _cholmodSparse;
  std::unique_ptr<cholmod_factor, CholmodFree> _cholmodFactor{
      nullptr, CholmodFree{&_cholmodCommon}};
  bool _blockOrdering{};
  MatrixStructure _matrixStructure;
  Eigen::VectorXi _scalarPermutation, _blockPermutation;

 public:
  void computeSymbolicDecomposition(const SparseBlockMatrix<MatrixType>& A) {
    double t = get_monotonic_time();

    A.fillBlockStructure(_matrixStructure);

    // get the ordering for the block matrix
    if (_blockPermutation.size() <
        _matrixStructure.n)  // double space if resizing
      _blockPermutation.resize(2 * _matrixStructure.n);

    int amdStatus =
        camd_order(_matrixStructure.n, _matrixStructure.Ap.get(),
                   _matrixStructure.Aii.get(), _blockPermutation.data(),
                   nullptr, nullptr, cmember->data());
    if (amdStatus != CAMD_OK) {
      ngx::Log(ngx::Error, "Error while computing ordering");
    }

    // blow up the permutation to the scalar matrix and extend to include the
    // additional blocks
    if (_scalarPermutation.size() == 0)
      _scalarPermutation.resize(_cholmodSparse->ncol);
    if (_scalarPermutation.size() < (int)_cholmodSparse->ncol)
      _scalarPermutation.resize(2 * _cholmodSparse->ncol);
    size_t scalarIdx = 0;
    for (int i = 0; i < _matrixStructure.n; ++i) {
      const int& p = _blockPermutation(i);
      int base = A.colBaseOfBlock(p);
      int nCols = A.colsOfBlock(p);
      for (int j = 0; j < nCols; ++j) _scalarPermutation(scalarIdx++) = base++;
    }

    for (; scalarIdx < _cholmodSparse->ncol;
         ++scalarIdx)  // extending for the additional blocks
      _scalarPermutation(scalarIdx) = scalarIdx;
    assert(scalarIdx == _cholmodSparse->ncol);

    // apply the ordering
    _cholmodCommon.nmethods = 1;
    _cholmodCommon.method[0].ordering = CHOLMOD_GIVEN;
    _cholmodFactor.reset(cholmod_analyze_p(_cholmodSparse.get(),
                                           _scalarPermutation.data(), nullptr,
                                           0, &_cholmodCommon));

    G2OBatchStatistics* globalStats = G2OBatchStatistics::globalStats();
    if (globalStats != nullptr)
      globalStats->timeSymbolicDecomposition = get_monotonic_time() - t;
  }

  void fillCholmodExt(const SparseBlockMatrix<MatrixType>& A, bool onlyValues) {
    int blockDimension = MatrixType::RowsAtCompileTime;
    assert(blockDimension > 0);
    // size_t origM = A.rows();
    size_t origN = A.cols();
    int additionalSpace = blockDimension * batchEveryN;
    size_t m = A.rows() + additionalSpace;
    size_t n = A.cols() + additionalSpace;

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
      size_t nzmax = A.nonZeros() + additionalSpace;
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

    int nz = 0;
    if (onlyValues)
      nz = A.fillCCS((double*)_cholmodSparse->x, true);
    else
      nz = A.fillCCS((int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
                     (double*)_cholmodSparse->x, true);

    int* cp = (int*)_cholmodSparse->p;
    int* ci = (int*)_cholmodSparse->i;
    double* cx = (double*)_cholmodSparse->x;

    cp = &cp[origN];
    ci = &ci[nz];
    cx = &cx[nz];

    // diagonal for the next blocks
    for (int i = 0; i < additionalSpace; ++i) {
      *cp++ = nz;
      *ci++ = origN + i;
      *cx++ = 1e-6;
      ++nz;
    }
    *cp = nz;
  }
};

G2O_END_NAMESPACE

#include <g2o/stuff/logger.h>
#include <g2o/suffix.h>

#endif
