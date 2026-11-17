// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/sparse_block_matrix.h>

#include <iostream>

using namespace std;
using namespace g2o;
using namespace Eigen;

typedef SparseBlockMatrix<MatrixXd> SparseBlockMatrixX;

std::ostream& operator<<(std::ostream& os,
                         const SparseBlockMatrixX::SparseMatrixBlock& m) {
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) cerr << m(i, j) << " ";
    cerr << endl;
  }
  return os;
}

int main(int argc, char** argv) {
  int rcol[] = {3, 6, 8, 12};
  int ccol[] = {2, 4, 13};
  cerr << "creation" << endl;
  SparseBlockMatrixX* M = new SparseBlockMatrixX(rcol, ccol, 4, 3);

  cerr << "block access" << endl;

  SparseBlockMatrixX::SparseMatrixBlock* b = M->block(0, 0, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i = 0; i < b->rows(); ++i)
    for (int j = 0; j < b->cols(); ++j) {
      (*b)(i, j) = i * b->cols() + j;
    }

  cerr << "block access 2" << endl;
  b = M->block(0, 2, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i = 0; i < b->rows(); ++i)
    for (int j = 0; j < b->cols(); ++j) {
      (*b)(i, j) = i * b->cols() + j;
    }

  b = M->block(3, 2, true);
  cerr << b->rows() << " " << b->cols() << endl;
  for (int i = 0; i < b->rows(); ++i)
    for (int j = 0; j < b->cols(); ++j) {
      (*b)(i, j) = i * b->cols() + j;
    }

  cerr << *M << endl;

  cerr << "SUM" << endl;

  SparseBlockMatrixX* Ms = 0;
  M->add(Ms);
  M->add(Ms);
  cerr << *Ms;

  SparseBlockMatrixX* Mt = 0;
  M->transpose(Mt);
  cerr << *Mt << endl;

  SparseBlockMatrixX* Mp = 0;
  M->multiply(Mp, Mt);
  cerr << *Mp << endl;

  int iperm[] = {3, 2, 1, 0};
  SparseBlockMatrixX* PMp = 0;

  Mp->symmPermutation(PMp, iperm, false);
  cerr << *PMp << endl;

  PMp->clear(true);
  Mp->block(3, 0)->fill(0.);
  Mp->symmPermutation(PMp, iperm, true);
  cerr << *PMp << endl;
}
