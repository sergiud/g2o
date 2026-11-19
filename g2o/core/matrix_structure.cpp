// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/matrix_structure.h>
#include <g2o/stuff/sparse_helper.h>

#include <memory>
#include <vector>

G2O_START_NAMESPACE

MatrixStructure::MatrixStructure() : n(0), m(0), maxN(0), maxNz(0) {}

MatrixStructure::~MatrixStructure() { free(); }

void MatrixStructure::alloc(int n_, int nz) {
  if (n == 0) {
    maxN = n = n_;
    maxNz = nz;
    Ap = std::make_unique<int[]>(maxN + 1);
    Aii = std::make_unique<int[]>(maxNz);
  } else {
    n = n_;
    if (maxNz < nz) {
      maxNz = 2 * nz;
      Aii = std::make_unique<int[]>(maxNz);
    }
    if (maxN < n) {
      maxN = 2 * n;
      Ap = std::make_unique<int[]>(maxN + 1);
    }
  }
}

void MatrixStructure::free() {
  n = 0;
  m = 0;
  maxN = 0;
  maxNz = 0;
  Aii.reset();
  Ap.reset();
}

bool MatrixStructure::write(const char* filename) const {
  // The structure holds no values, which the file lists as zeros.
  const std::vector<double> zeros(n > 0 ? Ap[n] : 0, 0.);
  return writeCCSMatrix(filename, m, n, Ap.get(), Aii.get(), zeros.data(),
                        /*upperTriangleSymmetric=*/true);
}

G2O_END_NAMESPACE
