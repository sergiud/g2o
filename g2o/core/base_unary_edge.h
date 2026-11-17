// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W.
// Burgard SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_BASE_UNARY_EDGE_H
#define G2O_BASE_UNARY_EDGE_H

#include <g2o/config.h>
#include <g2o/core/base_edge.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/stuff/logger.h>

#include <cassert>
#include <limits>

G2O_START_NAMESPACE

template <int D, typename E, typename VertexXi>
class BaseUnaryEdge : public BaseEdge<D, E> {
 public:
  static const int Dimension = BaseEdge<D, E>::Dimension;
  typedef typename BaseEdge<D, E>::Measurement Measurement;
  typedef VertexXi VertexXiType;
  typedef typename Eigen::Matrix<double, D, VertexXiType::Dimension,
                                 D == 1 ? Eigen::RowMajor
                                        : Eigen::ColMajor>::AlignedMapType
      JacobianXiOplusType;
  typedef typename BaseEdge<D, E>::ErrorVector ErrorVector;
  typedef typename BaseEdge<D, E>::InformationType InformationType;

  BaseUnaryEdge()
      : BaseEdge<D, E>(),
        _jacobianOplusXi(nullptr, D, VertexXiType::Dimension) {
    _vertices.resize(1);
  }

  virtual void resize(size_t size);

  virtual bool allVerticesFixed() const;

  virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

  virtual OptimizableGraph::Vertex* createVertex(int i);

  /**
   * Linearizes the oplus operator in the vertex, and stores
   * the result in temporary variables _jacobianOplusXi and _jacobianOplusXj
   */
  virtual void linearizeOplus();

  //! returns the result of the linearization in the manifold space for the node
  //! xi
  const JacobianXiOplusType& jacobianOplusXi() const {
    return _jacobianOplusXi;
  }

  virtual void constructQuadraticForm();

  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* to);

  virtual void mapHessianMemory(double*, int, int, bool) {
    assert(0 && "BaseUnaryEdge does not map memory of the Hessian");
  }

  using BaseEdge<D, E>::resize;
  using BaseEdge<D, E>::computeError;

 protected:
  using BaseEdge<D, E>::_measurement;
  using BaseEdge<D, E>::_information;
  using BaseEdge<D, E>::_error;
  using BaseEdge<D, E>::_vertices;
  using BaseEdge<D, E>::_dimension;

  JacobianXiOplusType _jacobianOplusXi;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_unary_edge.hpp"

G2O_END_NAMESPACE

#endif
