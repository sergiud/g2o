// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_MARGINAL_COVARIANCE_CHOLESKY_H
#define G2O_MARGINAL_COVARIANCE_CHOLESKY_H

#include <g2o/core/optimizable_graph.h>
#include <g2o/core/sparse_block_matrix.h>

#include <cassert>
#include <vector>

#include <unordered_map>

#include <g2o/core/g2o_core_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief computing the marginal covariance given a cholesky factor (lower triangle of the factor)
   */
  class G2O_CORE_API MarginalCovarianceCholesky {
    protected:
      /**
       * hash struct for storing the matrix elements needed to compute the covariance
       */
      typedef std::unordered_map<int, double>     LookupMap;
    
    public:
      MarginalCovarianceCholesky();
      ~MarginalCovarianceCholesky();

      /**
       * compute the marginal cov for the given block indices, write the result to the covBlocks memory (which has to
       * be provided by the caller).
       */
      void computeCovariance(double** covBlocks, const std::vector<int>& blockIndices);


      /**
       * compute the marginal cov for the given block indices, write the result in spinv).
       */
      void computeCovariance(SparseBlockMatrix<MatrixXD>& spinv, const std::vector<int>& rowBlockIndices, const std::vector< std::pair<int, int> >& blockIndices);


      /**
       * set the CCS representation of the cholesky factor along with the inverse permutation used to reduce the fill-in.
       * permInv might be 0, will then not permute the entries.
       *
       * The pointers provided by the user need to be still valid when calling computeCovariance(). The pointers
       * are owned by the caller, MarginalCovarianceCholesky does not free the pointers.
       */
      void setCholeskyFactor(int n, int* Lp, int* Li, double* Lx, int* permInv);

    protected:
      // information about the cholesky factor (lower triangle)
      int _n;           ///< L is an n X n matrix
      int* _Ap;         ///< column pointer of the CCS storage
      int* _Ai;         ///< row indices of the CCS storage
      double* _Ax;      ///< values of the cholesky factor
      int* _perm;       ///< permutation of the cholesky factor. Variable re-ordering for better fill-in

      LookupMap _map;             ///< hash look up table for the already computed entries
      std::vector<double> _diag;  ///< cache 1 / H_ii to avoid recalculations

      //! compute the index used for hashing
      int computeIndex(int r, int c) const { /*assert(r <= c);*/ return r*_n + c;}
      /**
       * compute one entry in the covariance, r and c are values after applying the permutation, and upper triangular.
       * May issue recursive calls to itself to compute the missing values.
       */
      double computeEntry(int r, int c);
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
