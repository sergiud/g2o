// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W.
// Burgard SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_BASE_MULTI_EDGE_H
#define G2O_BASE_MULTI_EDGE_H

#include <g2o/config.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/robust_kernel.h>

#include <Eigen/StdVector>
#include <cstddef>
#include <g2o/core/dynamic_aligned_buffer.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief base class to represent an edge connecting an arbitrary number of
 * nodes
 *
 * D - Dimension of the measurement
 * E - type to represent the measurement
 */
template <int D, typename E>
class BaseMultiEdge : public BaseEdge<D, E> {
 public:
  /**
   * \brief helper for mapping the Hessian memory of the upper triangular block
   */
  struct HessianHelper {
    Eigen::Map<MatrixXD> matrix;  ///< the mapped memory
    bool transposed;              ///< the block has to be transposed
    HessianHelper() : matrix(nullptr, 0, 0), transposed(false) {}
  };

 public:
  static const int Dimension = BaseEdge<D, E>::Dimension;
  typedef typename BaseEdge<D, E>::Measurement Measurement;
  typedef MatrixXD::MapType JacobianType;
  typedef typename BaseEdge<D, E>::ErrorVector ErrorVector;
  typedef typename BaseEdge<D, E>::InformationType InformationType;
  typedef ConditionallyAligned<MatrixXD>::MapType HessianBlockType;

  BaseMultiEdge() : BaseEdge<D, E>() {}

  virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

  /**
   * Linearizes the oplus operator in the vertex, and stores
   * the result in temporary variable vector _jacobianOplus
   */
  virtual void linearizeOplus();

  virtual void resize(size_t size);

  virtual bool allVerticesFixed() const;

  virtual void constructQuadraticForm();

  virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

  using BaseEdge<D, E>::computeError;

 protected:
  using BaseEdge<D, E>::_measurement;
  using BaseEdge<D, E>::_information;
  using BaseEdge<D, E>::_error;
  using BaseEdge<D, E>::_vertices;
  using BaseEdge<D, E>::_dimension;

  std::vector<HessianHelper> _hessian;
  std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> >
      _jacobianOplus;  ///< jacobians of the edge (w.r.t. oplus)

  void computeQuadraticForm(const InformationType& omega,
                            const ErrorVector& weightedError);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// PARTIAL TEMPLATE SPECIALIZATION
template <typename E>
class BaseMultiEdge<-1, E> : public BaseEdge<-1, E> {
 public:
  /**
   * \brief helper for mapping the Hessian memory of the upper triangular block
   */
  struct HessianHelper {
    Eigen::Map<MatrixXD> matrix;  ///< the mapped memory
    bool transposed;              ///< the block has to be transposed
    HessianHelper() : matrix(nullptr, 0, 0), transposed(false) {}
  };

 public:
  static const int Dimension = BaseEdge<-1, E>::Dimension;
  typedef typename BaseEdge<-1, E>::Measurement Measurement;
  typedef MatrixXD::MapType JacobianType;
  typedef typename BaseEdge<-1, E>::ErrorVector ErrorVector;
  typedef typename BaseEdge<-1, E>::InformationType InformationType;
  typedef ConditionallyAligned<MatrixXD>::MapType HessianBlockType;

  BaseMultiEdge() : BaseEdge<-1, E>() {
    // this->_variableSize = true;
  }

  virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

  /**
   * Linearizes the oplus operator in the vertex, and stores
   * the result in temporary variable vector _jacobianOplus
   */
  virtual void linearizeOplus();

  virtual void resize(size_t size);

  virtual bool allVerticesFixed() const;

  virtual void constructQuadraticForm();

  virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

  using BaseEdge<-1, E>::computeError;

 protected:
  using BaseEdge<-1, E>::_measurement;
  using BaseEdge<-1, E>::_information;
  using BaseEdge<-1, E>::_error;
  using BaseEdge<-1, E>::_vertices;
  using BaseEdge<-1, E>::_dimension;

  std::vector<HessianHelper> _hessian;
  std::vector<JacobianType, Eigen::aligned_allocator<JacobianType> >
      _jacobianOplus;  ///< jacobians of the edge (w.r.t. oplus)

  void computeQuadraticForm(const InformationType& omega,
                            const ErrorVector& weightedError);

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

#include "base_multi_edge.hpp"

G2O_END_NAMESPACE

#endif
