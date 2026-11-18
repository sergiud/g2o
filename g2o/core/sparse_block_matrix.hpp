// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/prefix.h>

#include <memory>
#include <utility>

G2O_START_NAMESPACE

namespace {
struct TripletEntry {
  int r, c;
  double x;
  TripletEntry(int r_, int c_, double x_) : r(r_), c(c_), x(x_) {}
};
struct TripletColSort {
  bool operator()(const TripletEntry& e1, const TripletEntry& e2) const {
    return e1.c < e2.c || (e1.c == e2.c && e1.r < e2.r);
  }
};
/** Helper class to sort pair based on first elem */
template <class T1, class T2, class Pred = std::less<T1>>
struct CmpPairFirst {
  bool operator()(const std::pair<T1, T2>& left,
                  const std::pair<T1, T2>& right) {
    return Pred()(left.first, right.first);
  }
};
}  // namespace

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix(const int* rbi, const int* cbi,
                                                 int rb, int cb)
    : _rowBlockIndices(rbi, rbi + rb),
      _colBlockIndices(cbi, cbi + cb),
      _blockCols(cb) {}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::SparseBlockMatrix() : _blockCols(0) {}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::clear(bool dealloc) {
#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) if (_blockCols.size() > 100)
#endif
  for (int i = 0; i < static_cast<int>(_blockCols.size()); ++i) {
    if (dealloc) {
      _blockCols[i].clear();
      continue;
    }
    for (auto& it : _blockCols[i]) {
      it.second->setZero();
    }
  }
}

template <class MatrixType>
SparseBlockMatrix<MatrixType>::~SparseBlockMatrix() = default;

template <class MatrixType>
typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock*
SparseBlockMatrix<MatrixType>::block(int r, int c, bool alloc) {
  auto it = _blockCols[c].find(r);
  if (it != _blockCols[c].end()) return it->second.get();
  // The matrix owns its blocks, so it creates a missing block regardless of
  // alloc.
  static_cast<void>(alloc);
  int rb = rowsOfBlock(r);
  int cb = colsOfBlock(c);
  auto block = std::make_unique<SparseMatrixBlock>(rb, cb);
  block->setZero();
  SparseMatrixBlock* created = block.get();
  _blockCols[c].emplace(r, std::move(block));
  return created;
}

template <class MatrixType>
const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock*
SparseBlockMatrix<MatrixType>::block(int r, int c) const {
  auto it = _blockCols[c].find(r);
  if (it == _blockCols[c].end()) return nullptr;
  return it->second.get();
}

template <class MatrixType>
std::unique_ptr<SparseBlockMatrix<MatrixType>>
SparseBlockMatrix<MatrixType>::clone() const {
  auto ret = std::make_unique<SparseBlockMatrix>(
      &_rowBlockIndices[0], &_colBlockIndices[0], _rowBlockIndices.size(),
      _colBlockIndices.size());
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      ret->_blockCols[i].emplace(
          it->first, std::make_unique<SparseMatrixBlock>(*it->second));
    }
  }
  return ret;
}

template <class MatrixType>
template <class MatrixTransposedType>
bool SparseBlockMatrix<MatrixType>::transpose(
    std::unique_ptr<SparseBlockMatrix<MatrixTransposedType>>& dest) const {
  if (!dest) {
    dest = std::make_unique<SparseBlockMatrix<MatrixTransposedType>>(
        &_colBlockIndices[0], &_rowBlockIndices[0], _colBlockIndices.size(),
        _rowBlockIndices.size());
  } else {
    if (_rowBlockIndices.size() != dest->_colBlockIndices.size()) return false;
    if (_colBlockIndices.size() != dest->_rowBlockIndices.size()) return false;
    for (size_t i = 0; i < _rowBlockIndices.size(); ++i) {
      if (_rowBlockIndices[i] != dest->_colBlockIndices[i]) return false;
    }
    for (size_t i = 0; i < _colBlockIndices.size(); ++i) {
      if (_colBlockIndices[i] != dest->_rowBlockIndices[i]) return false;
    }
  }

  for (size_t i = 0; i < _blockCols.size(); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s =
          it->second.get();
      typename SparseBlockMatrix<MatrixTransposedType>::SparseMatrixBlock* d =
          dest->block(i, it->first, true);
      *d = s->transpose();
    }
  }
  return true;
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::add(
    std::unique_ptr<SparseBlockMatrix>& dest) const {
  if (!dest) {
    dest = std::make_unique<SparseBlockMatrix>(
        &_rowBlockIndices[0], &_colBlockIndices[0], _rowBlockIndices.size(),
        _colBlockIndices.size());
  } else {
    if (_rowBlockIndices.size() != dest->_rowBlockIndices.size()) return false;
    if (_colBlockIndices.size() != dest->_colBlockIndices.size()) return false;
    for (size_t i = 0; i < _rowBlockIndices.size(); ++i) {
      if (_rowBlockIndices[i] != dest->_rowBlockIndices[i]) return false;
    }
    for (size_t i = 0; i < _colBlockIndices.size(); ++i) {
      if (_colBlockIndices[i] != dest->_colBlockIndices[i]) return false;
    }
  }
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s =
          it->second.get();
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* d =
          dest->block(it->first, i, true);
      (*d) += *s;
    }
  }
  return true;
}

template <class MatrixType>
template <class MatrixResultType, class MatrixFactorType>
bool SparseBlockMatrix<MatrixType>::multiply(
    std::unique_ptr<SparseBlockMatrix<MatrixResultType>>& dest,
    const SparseBlockMatrix<MatrixFactorType>* M) const {
  // sanity check
  if (_colBlockIndices.size() != M->_rowBlockIndices.size()) return false;
  for (size_t i = 0; i < _colBlockIndices.size(); ++i) {
    if (_colBlockIndices[i] != M->_rowBlockIndices[i]) return false;
  }
  if (!dest) {
    dest = std::make_unique<SparseBlockMatrix<MatrixResultType>>(
        &_rowBlockIndices[0], &(M->_colBlockIndices[0]),
        _rowBlockIndices.size(), M->_colBlockIndices.size());
  }
  for (size_t i = 0; i < M->_blockCols.size(); ++i) {
    for (typename SparseBlockMatrix<
             MatrixFactorType>::IntBlockMap::const_iterator it =
             M->_blockCols[i].begin();
         it != M->_blockCols[i].end(); ++it) {
      // look for a non-zero block in a row of column it
      int colM = i;
      const typename SparseBlockMatrix<MatrixFactorType>::SparseMatrixBlock* b =
          it->second.get();
      typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator rbt =
          _blockCols[it->first].begin();
      while (rbt != _blockCols[it->first].end()) {
        // int colA=it->first;
        int rowA = rbt->first;
        typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
            rbt->second.get();
        typename SparseBlockMatrix<MatrixResultType>::SparseMatrixBlock* c =
            dest->block(rowA, colM, true);
        assert(c->rows() == a->rows());
        assert(c->cols() == b->cols());
        ++rbt;
        (*c) += (*a) * (*b);
      }
    }
  }
  return false;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::multiply(double* dest,
                                             const double* src) const {
  assert(dest != nullptr && "the caller provides the destination");

  // map the memory by Eigen
  Eigen::Map<VectorXD> destVec(dest, rows());
  const Eigen::Map<const VectorXD> srcVec(src, cols());

  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int srcOffset = i != 0u ? _colBlockIndices[i - 1] : 0;

    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second.get();
      int destOffset = it->first ? _rowBlockIndices[it->first - 1] : 0;
      // destVec += *a * srcVec (according to the sub-vector parts)
      internal::template axpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::multiplySymmetricUpperTriangle(
    double* dest, const double* src) const {
  assert(dest != nullptr && "the caller provides the destination");

  // map the memory by Eigen
  Eigen::Map<VectorXD> destVec(dest, rows());
  const Eigen::Map<const VectorXD> srcVec(src, cols());

  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int srcOffset = colBaseOfBlock(i);
    for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second.get();
      int destOffset = rowBaseOfBlock(it->first);
      if (destOffset > srcOffset)  // only upper triangle
        break;
      // destVec += *a * srcVec (according to the sub-vector parts)
      internal::template axpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
      if (destOffset < srcOffset)
        internal::template atxpy<
            typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
            *a, srcVec, destOffset, destVec, srcOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::rightMultiply(double* dest,
                                                  const double* src) const {
  assert(dest != nullptr && "the caller provides the destination");
  int destSize = cols();

  // map the memory by Eigen
  Eigen::Map<VectorXD> destVec(dest, destSize);
  Eigen::Map<const VectorXD> srcVec(src, rows());

#ifdef G2O_OPENMP
#pragma omp parallel for default(shared) schedule(dynamic, 10)
#endif
  for (int i = 0; i < static_cast<int>(_blockCols.size()); ++i) {
    int destOffset = colBaseOfBlock(i);
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second.get();
      int srcOffset = rowBaseOfBlock(it->first);
      // destVec += *a.transpose() * srcVec (according to the sub-vector parts)
      internal::template atxpy<
          typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock>(
          *a, srcVec, srcOffset, destVec, destOffset);
    }
  }
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::scale(double a_) {
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
          it->second.get();
      *a *= a_;
    }
  }
}

template <class MatrixType>
std::unique_ptr<SparseBlockMatrix<MatrixType>>
SparseBlockMatrix<MatrixType>::slice(int rmin, int rmax, int cmin,
                                     int cmax) const {
  int m = rmax - rmin;
  int n = cmax - cmin;
  std::vector<int> rowIdx(m);
  rowIdx[0] = rowsOfBlock(rmin);
  for (int i = 1; i < m; ++i) {
    rowIdx[i] = rowIdx[i - 1] + rowsOfBlock(rmin + i);
  }

  std::vector<int> colIdx(n);
  colIdx[0] = colsOfBlock(cmin);
  for (int i = 1; i < n; ++i) {
    colIdx[i] = colIdx[i - 1] + colsOfBlock(cmin + i);
  }
  auto s =
      std::make_unique<SparseBlockMatrix>(rowIdx.data(), colIdx.data(), m, n);
  for (int i = 0; i < n; ++i) {
    int mc = cmin + i;
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[mc].begin();
         it != _blockCols[mc].end(); ++it) {
      if (it->first >= rmin && it->first < rmax) {
        s->_blockCols[i].emplace(
            it->first - rmin, std::make_unique<SparseMatrixBlock>(*it->second));
      }
    }
  }
  return s;
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeroBlocks() const {
  size_t count = 0;
  for (size_t i = 0; i < _blockCols.size(); ++i) count += _blockCols[i].size();
  return count;
}

template <class MatrixType>
size_t SparseBlockMatrix<MatrixType>::nonZeros() const {
  if (MatrixType::SizeAtCompileTime != Eigen::Dynamic) {
    size_t nnz = nonZeroBlocks() * MatrixType::SizeAtCompileTime;
    return nnz;
  } else {
    size_t count = 0;
    for (size_t i = 0; i < _blockCols.size(); ++i) {
      for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* a =
            it->second.get();
        count += a->cols() * a->rows();
      }
    }
    return count;
  }
}

template <class MatrixType>
std::ostream& operator<<(std::ostream& os,
                         const SparseBlockMatrix<MatrixType>& m) {
  os << "RBI: " << m.rowBlockIndices().size();
  for (size_t i = 0; i < m.rowBlockIndices().size(); ++i)
    os << " " << m.rowBlockIndices()[i];
  os << std::endl;
  os << "CBI: " << m.colBlockIndices().size();
  for (size_t i = 0; i < m.colBlockIndices().size(); ++i)
    os << " " << m.colBlockIndices()[i];
  os << std::endl;

  for (size_t i = 0; i < m.blockCols().size(); ++i) {
    for (auto it = m.blockCols()[i].begin(); it != m.blockCols()[i].end();
         ++it) {
      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
          it->second.get();
      os << "BLOCK: " << it->first << " " << i << std::endl;
      os << *b << std::endl;
    }
  }
  return os;
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::symmPermutation(
    std::unique_ptr<SparseBlockMatrix<MatrixType>>& dest, const int* pinv,
    bool upperTriangle) const {
  // compute the permuted version of the new row/column layout
  size_t n = _rowBlockIndices.size();
  // computed the block sizes
  int blockSizes[_rowBlockIndices.size()];
  blockSizes[0] = _rowBlockIndices[0];
  for (size_t i = 1; i < n; ++i) {
    blockSizes[i] = _rowBlockIndices[i] - _rowBlockIndices[i - 1];
  }
  // permute them
  int pBlockIndices[_rowBlockIndices.size()];
  for (size_t i = 0; i < n; ++i) {
    pBlockIndices[pinv[i]] = blockSizes[i];
  }
  for (size_t i = 1; i < n; ++i) {
    pBlockIndices[i] += pBlockIndices[i - 1];
  }
  // allocate C, or check the structure;
  if (!dest) {
    dest =
        std::make_unique<SparseBlockMatrix>(pBlockIndices, pBlockIndices, n, n);
  } else {
    if (dest->_rowBlockIndices.size() != n) return false;
    if (dest->_colBlockIndices.size() != n) return false;
    for (size_t i = 0; i < n; ++i) {
      if (dest->_rowBlockIndices[i] != pBlockIndices[i]) return false;
      if (dest->_colBlockIndices[i] != pBlockIndices[i]) return false;
    }
    dest->clear();
  }
  // now ready to permute the columns
  for (size_t i = 0; i < n; ++i) {
    // cerr << PVAR(i) <<  " ";
    int pi = pinv[i];
    for (typename SparseBlockMatrix<MatrixType>::IntBlockMap::const_iterator
             it = _blockCols[i].begin();
         it != _blockCols[i].end(); ++it) {
      int pj = pinv[it->first];

      const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* s =
          it->second.get();

      typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b = 0;
      if (!upperTriangle || pj <= pi) {
        b = dest->block(pj, pi, true);
        assert(b->cols() == s->cols());
        assert(b->rows() == s->rows());
        *b = *s;
      } else {
        b = dest->block(pi, pj, true);
        assert(b);
        assert(b->rows() == s->cols());
        assert(b->cols() == s->rows());
        *b = s->transpose();
      }
    }
    // cerr << endl;
    //  within each row,
  }
  return true;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(double* Cx,
                                           bool upperTriangle) const {
  assert(Cx && "Target destination is NULL");
  double* CxStart = Cx;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int cstart = i != 0u ? _colBlockIndices[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; ++c) {
      for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second.get();
        int rstart = it->first ? _rowBlockIndices[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart) elemsToCopy = c + 1;
        memcpy(Cx, b->data() + c * b->rows(), elemsToCopy * sizeof(double));
        Cx += elemsToCopy;
      }
    }
  }
  return Cx - CxStart;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillCCS(int* Cp, int* Ci, double* Cx,
                                           bool upperTriangle) const {
  assert(Cp && Ci && Cx && "Target destination is NULL");
  int nz = 0;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    int cstart = i != 0u ? _colBlockIndices[i - 1] : 0;
    int csize = colsOfBlock(i);
    for (int c = 0; c < csize; ++c) {
      *Cp = nz;
      for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
        const typename SparseBlockMatrix<MatrixType>::SparseMatrixBlock* b =
            it->second.get();
        int rstart = it->first ? _rowBlockIndices[it->first - 1] : 0;

        int elemsToCopy = b->rows();
        if (upperTriangle && rstart == cstart) elemsToCopy = c + 1;
        for (int r = 0; r < elemsToCopy; ++r) {
          *Cx++ = (*b)(r, c);
          *Ci++ = rstart++;
          ++nz;
        }
      }
      ++Cp;
    }
  }
  *Cp = nz;
  return nz;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::fillBlockStructure(
    MatrixStructure& ms) const {
  int n = _colBlockIndices.size();
  int nzMax = (int)nonZeroBlocks();

  ms.alloc(n, nzMax);
  ms.m = _rowBlockIndices.size();

  int nz = 0;
  int* Cp = ms.Ap.get();
  int* Ci = ms.Aii.get();
  for (int i = 0; i < static_cast<int>(_blockCols.size()); ++i) {
    *Cp = nz;
    const int& c = i;
    for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const int& r = it->first;
      if (r <= c) {
        *Ci++ = r;
        ++nz;
      }
    }
    Cp++;
  }
  *Cp = nz;
  assert(nz <= nzMax);
}

template <class MatrixType>
bool SparseBlockMatrix<MatrixType>::writeOctave(const char* filename,
                                                bool upperTriangle) const {
  std::string name = filename;
  std::string::size_type lastDot = name.find_last_of('.');
  if (lastDot != std::string::npos) name = name.substr(0, lastDot);

  std::vector<TripletEntry> entries;
  for (size_t i = 0; i < _blockCols.size(); ++i) {
    const auto& c = i;
    for (auto it = _blockCols[i].begin(); it != _blockCols[i].end(); ++it) {
      const int& r = it->first;
      const MatrixType& m = *(it->second);
      for (int cc = 0; cc < m.cols(); ++cc)
        for (int rr = 0; rr < m.rows(); ++rr) {
          int aux_r = rowBaseOfBlock(r) + rr;
          int aux_c = colBaseOfBlock(c) + cc;
          entries.push_back(TripletEntry(aux_r, aux_c, m(rr, cc)));
          if (upperTriangle && r != c) {
            entries.push_back(TripletEntry(aux_c, aux_r, m(rr, cc)));
          }
        }
    }
  }

  int nz = entries.size();
  std::sort(entries.begin(), entries.end(), TripletColSort());

  std::ofstream fout(filename);
  fout << "# name: " << name << std::endl;
  fout << "# type: sparse matrix" << std::endl;
  fout << "# nnz: " << nz << std::endl;
  fout << "# rows: " << rows() << std::endl;
  fout << "# columns: " << cols() << std::endl;
  fout << std::setprecision(9) << std::fixed << std::endl;

  for (std::vector<TripletEntry>::const_iterator it = entries.begin();
       it != entries.end(); ++it) {
    const TripletEntry& entry = *it;
    fout << entry.r + 1 << " " << entry.c + 1 << " " << entry.x << std::endl;
  }
  return fout.good();
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillSparseBlockMatrixCCS(
    SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
  blockCCS.blockCols().resize(blockCols().size());
  int numblocks = 0;
  for (size_t i = 0; i < blockCols().size(); ++i) {
    const IntBlockMap& row = blockCols()[i];
    typename SparseBlockMatrixCCS<MatrixType>::SparseColumn& dest =
        blockCCS.blockCols()[i];
    dest.clear();
    dest.reserve(row.size());
    for (auto it = row.begin(); it != row.end(); ++it) {
      dest.push_back(typename SparseBlockMatrixCCS<MatrixType>::RowBlock(
          it->first, it->second.get()));
      ++numblocks;
    }
  }
  return numblocks;
}

template <class MatrixType>
int SparseBlockMatrix<MatrixType>::fillSparseBlockMatrixCCSTransposed(
    SparseBlockMatrixCCS<MatrixType>& blockCCS) const {
  blockCCS.blockCols().clear();
  blockCCS.blockCols().resize(_rowBlockIndices.size());
  int numblocks = 0;
  for (size_t i = 0; i < blockCols().size(); ++i) {
    const IntBlockMap& row = blockCols()[i];
    for (auto it = row.begin(); it != row.end(); ++it) {
      typename SparseBlockMatrixCCS<MatrixType>::SparseColumn& dest =
          blockCCS.blockCols()[it->first];
      dest.push_back(typename SparseBlockMatrixCCS<MatrixType>::RowBlock(
          i, it->second.get()));
      ++numblocks;
    }
  }
  return numblocks;
}

template <class MatrixType>
void SparseBlockMatrix<MatrixType>::takePatternFromHash(
    SparseBlockMatrixHashMap<MatrixType>& hashMatrix) {
  // sort the sparse columns and add them to the map structures by
  // exploiting that we are inserting a sorted structure
  typedef std::pair<int, std::unique_ptr<MatrixType>> SparseColumnPair;
  typedef typename SparseBlockMatrixHashMap<MatrixType>::SparseColumn
      HashSparseColumn;
  for (size_t i = 0; i < hashMatrix.blockCols().size(); ++i) {
    // prepare a temporary vector for sorting
    HashSparseColumn& column = hashMatrix.blockCols()[i];
    if (column.empty()) continue;
    std::vector<SparseColumnPair> sparseRowSorted;  // temporary structure
    sparseRowSorted.reserve(column.size());
    for (auto& it : column) {
      sparseRowSorted.emplace_back(it.first, std::move(it.second));
    }
    std::sort(sparseRowSorted.begin(), sparseRowSorted.end(),
              [](const SparseColumnPair& left, const SparseColumnPair& right) {
                return left.first < right.first;
              });
    // try to free some memory early
    HashSparseColumn aux;
    std::swap(aux, column);
    // now insert sorted vector to the std::map structure
    IntBlockMap& destColumnMap = blockCols()[i];
    for (auto& it : sparseRowSorted) {
      destColumnMap.insert(destColumnMap.end(), std::move(it));
    }
  }
}

G2O_END_NAMESPACE

#include <g2o/suffix.h>
