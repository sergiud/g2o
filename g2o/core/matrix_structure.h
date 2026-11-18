// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_MATRIX_STRUCTURE_H
#define G2O_MATRIX_STRUCTURE_H

#include <g2o/core/abi.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/prefix.h>

#include <memory>

G2O_START_NAMESPACE

/**
 * \brief representing the structure of a matrix in column compressed structure
 * (only the upper triangular part of the matrix)
 */
class G2O_CORE_API MatrixStructure {
 public:
  MatrixStructure();
  ~MatrixStructure();
  /**
   * allocate space for the Matrix Structure. You may call this on an already
   * allocated struct, it will then reallocate the memory + additional space
   * (double the required space).
   */
  void alloc(int n_, int nz);

  void free();

  /**
   * Write the matrix pattern to a file. File is also loadable by octave, e.g.,
   * then use spy(matrix)
   */
  bool write(const char* filename) const;

  int n;                       ///< A is m-by-n.  n must be >= 0.
  int m;                       ///< A is m-by-n.  m must be >= 0.
  std::unique_ptr<int[]> Ap;   ///< column pointers for A, of size n+1
  std::unique_ptr<int[]> Aii;  ///< row indices of A, of size nz = Ap [n]

  //! max number of non-zeros blocks
  int nzMax() const { return maxNz; }

 protected:
  int maxN;   ///< size of the allocated memory
  int maxNz;  ///< size of the allocated memory
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
