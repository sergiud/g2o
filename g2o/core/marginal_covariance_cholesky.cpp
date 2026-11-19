// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/marginal_covariance_cholesky.h>

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <utility>
#include <vector>
using namespace std;

G2O_START_NAMESPACE

struct MatrixElem {
  int r, c;
  MatrixElem(int r_, int c_) : r(r_), c(c_) {}
  bool operator<(const MatrixElem& other) const {
    return c > other.c || (c == other.c && r > other.r);
  }
};

MarginalCovarianceCholesky::MarginalCovarianceCholesky()
    : _n(0), _Ap(nullptr), _Ai(nullptr), _Ax(nullptr), _perm(nullptr) {}

MarginalCovarianceCholesky::~MarginalCovarianceCholesky() = default;

void MarginalCovarianceCholesky::setCholeskyFactor(int n, int* Lp, int* Li,
                                                   double* Lx, int* permInv) {
  _n = n;
  _Ap = Lp;
  _Ai = Li;
  _Ax = Lx;
  _perm = permInv;

  // pre-compute reciprocal values of the diagonal of L
  _diag.resize(n);
  for (int r = 0; r < n; ++r) {
    const int& sc = _Ap[r];  // L is lower triangular, thus the first elem in
                             // the column is the diagonal entry
    assert(r == _Ai[sc] && "Error in CCS storage of L");
    _diag[r] = 1.0 / _Ax[sc];
  }
}

double MarginalCovarianceCholesky::computeEntry(int r, int c) {
  assert(r <= c);
  // Each entry (r, c) depends on the entries of the rows below r in column r
  // of the factor, so the dependencies end at the last row.
  std::vector<std::pair<int, int>> pending{{r, c}};
  while (!pending.empty()) {
    const auto [row, col] = pending.back();
    if (_map.contains(computeIndex(row, col))) {
      pending.pop_back();
      continue;
    }

    // sum over column row while skipping the element on the diagonal
    const int sc = _Ap[row];
    const int ec = _Ap[row + 1];
    bool ready = true;
    double s = 0.;
    for (int j = sc + 1; j < ec; ++j) {
      const int rr = _Ai[j];
      const int depRow = rr < col ? rr : col;
      const int depCol = rr < col ? col : rr;
      const auto found = _map.find(computeIndex(depRow, depCol));
      if (found == _map.end()) {
        pending.emplace_back(depRow, depCol);
        ready = false;
      } else if (ready) {
        s += found->second * _Ax[j];
      }
    }
    if (!ready) {
      continue;
    }

    double result;
    if (row == col) {
      const double& diagElem = _diag[row];
      result = diagElem * (diagElem - s);
    } else {
      result = -s * _diag[row];
    }
    _map[computeIndex(row, col)] = result;
    pending.pop_back();
  }
  return _map[computeIndex(r, c)];
}

void MarginalCovarianceCholesky::computeCovariance(
    double** covBlocks, const std::vector<int>& blockIndices) {
  _map.clear();
  int base = 0;
  vector<MatrixElem> elemsToCompute;
  for (int nbase : blockIndices) {
    int vdim = nbase - base;
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = _perm != nullptr ? _perm[rr + base]
                                 : rr + base;  // apply permutation
        int c = _perm != nullptr ? _perm[cc + base] : cc + base;
        if (r > c)  // make sure it's still upper triangular after applying the
                    // permutation
          swap(r, c);
        elemsToCompute.emplace_back(r, c);
      }
    base = nbase;
  }

  // sort the elems to reduce the recursive calls
  sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (auto& me : elemsToCompute) {
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance for the vertices, by writing to the blocks
  // memory
  base = 0;
  for (size_t i = 0; i < blockIndices.size(); ++i) {
    int nbase = blockIndices[i];
    int vdim = nbase - base;
    double* cov = covBlocks[i];
    for (int rr = 0; rr < vdim; ++rr)
      for (int cc = rr; cc < vdim; ++cc) {
        int r = _perm != nullptr ? _perm[rr + base]
                                 : rr + base;  // apply permutation
        int c = _perm != nullptr ? _perm[cc + base] : cc + base;
        if (r > c)  // upper triangle
          swap(r, c);
        const std::int64_t idx = computeIndex(r, c);
        LookupMap::const_iterator foundIt = _map.find(idx);
        assert(foundIt != _map.end());
        cov[rr * vdim + cc] = foundIt->second;
        if (rr != cc) cov[cc * vdim + rr] = foundIt->second;
      }
    base = nbase;
  }
}

void MarginalCovarianceCholesky::computeCovariance(
    SparseBlockMatrix<MatrixXD>& spinv, const std::vector<int>& rowBlockIndices,
    const std::vector<std::pair<int, int>>& blockIndices) {
  // allocate the sparse
  spinv = SparseBlockMatrix<MatrixXD>(&rowBlockIndices[0], &rowBlockIndices[0],
                                      rowBlockIndices.size(),
                                      rowBlockIndices.size());
  _map.clear();
  vector<MatrixElem> elemsToCompute;
  for (const auto& blockIndice : blockIndices) {
    int blockRow = blockIndice.first;
    int blockCol = blockIndice.second;
    assert(blockRow >= 0);
    assert(blockRow < (int)rowBlockIndices.size());
    assert(blockCol >= 0);
    assert(blockCol < (int)rowBlockIndices.size());

    int rowBase = spinv.rowBaseOfBlock(blockRow);
    int colBase = spinv.colBaseOfBlock(blockCol);

    MatrixXD* block = spinv.block(blockRow, blockCol, true);
    assert(block);
    for (int iRow = 0; iRow < block->rows(); ++iRow)
      for (int iCol = 0; iCol < block->cols(); ++iCol) {
        int rr = rowBase + iRow;
        int cc = colBase + iCol;
        int r = _perm != nullptr ? _perm[rr] : rr;  // apply permutation
        int c = _perm != nullptr ? _perm[cc] : cc;
        if (r > c) swap(r, c);
        elemsToCompute.emplace_back(r, c);
      }
  }

  // sort the elems to reduce the number of recursive calls
  sort(elemsToCompute.begin(), elemsToCompute.end());

  // compute the inverse elements we need
  for (auto& me : elemsToCompute) {
    computeEntry(me.r, me.c);
  }

  // set the marginal covariance
  for (const auto& blockIndice : blockIndices) {
    int blockRow = blockIndice.first;
    int blockCol = blockIndice.second;
    int rowBase = spinv.rowBaseOfBlock(blockRow);
    int colBase = spinv.colBaseOfBlock(blockCol);

    MatrixXD* block = spinv.block(blockRow, blockCol);
    assert(block);
    for (int iRow = 0; iRow < block->rows(); ++iRow)
      for (int iCol = 0; iCol < block->cols(); ++iCol) {
        int rr = rowBase + iRow;
        int cc = colBase + iCol;
        int r = _perm != nullptr ? _perm[rr] : rr;  // apply permutation
        int c = _perm != nullptr ? _perm[cc] : cc;
        if (r > c) swap(r, c);
        const std::int64_t idx = computeIndex(r, c);
        LookupMap::const_iterator foundIt = _map.find(idx);
        assert(foundIt != _map.end());
        (*block)(iRow, iCol) = foundIt->second;
      }
  }
}

G2O_END_NAMESPACE
