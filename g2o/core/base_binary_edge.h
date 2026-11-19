// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// SPDX-FileCopyrightText: 2017-2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_BASE_BINARY_EDGE_H
#define G2O_BASE_BINARY_EDGE_H

#include <g2o/core/base_edge.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/stuff/logger.h>

#include <limits>
#include <memory>

G2O_START_NAMESPACE

template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseBinaryEdge : public BaseEdge<D, E> {
 public:
  typedef VertexXi VertexXiType;
  typedef VertexXj VertexXjType;

  static const int Di = VertexXiType::Dimension;
  static const int Dj = VertexXjType::Dimension;

  static const int Dimension = BaseEdge<D, E>::Dimension;
  typedef typename BaseEdge<D, E>::Measurement Measurement;
  typedef typename Eigen::Matrix<
      double, D, Di, D == 1 ? Eigen::RowMajor : Eigen::ColMajor>::AlignedMapType
      JacobianXiOplusType;
  typedef typename Eigen::Matrix<
      double, D, Dj, D == 1 ? Eigen::RowMajor : Eigen::ColMajor>::AlignedMapType
      JacobianXjOplusType;
  typedef typename BaseEdge<D, E>::ErrorVector ErrorVector;
  typedef typename BaseEdge<D, E>::InformationType InformationType;

  typedef Eigen::Map<Eigen::Matrix<double, Di, Dj,
                                   Di == 1 ? Eigen::RowMajor : Eigen::ColMajor>>
      HessianBlockType;
  typedef Eigen::Map<Eigen::Matrix<double, Dj, Di,
                                   Dj == 1 ? Eigen::RowMajor : Eigen::ColMajor>>
      HessianBlockTransposedType;

  BaseBinaryEdge()
      : BaseEdge<D, E>(),
        _hessianRowMajor(false),
        _hessian(nullptr, VertexXiType::Dimension,
                 VertexXjType::Dimension),  // HACK we map to the null pointer
                                            // for initializing the Maps
        _hessianTransposed(nullptr, VertexXjType::Dimension,
                           VertexXiType::Dimension),
        _jacobianOplusXi(nullptr, D, Di),
        _jacobianOplusXj(nullptr, D, Dj) {
    _vertices.resize(2);
  }

  std::unique_ptr<OptimizableGraph::Vertex> createFrom() override;
  std::unique_ptr<OptimizableGraph::Vertex> createTo() override;
  std::unique_ptr<OptimizableGraph::Vertex> createVertex(int i) override;

  virtual void resize(size_t size);

  virtual bool allVerticesFixed() const;

  virtual void linearizeOplus(JacobianWorkspace& jacobianWorkspace);

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
  //! returns the result of the linearization in the manifold space for the node
  //! xj
  const JacobianXjOplusType& jacobianOplusXj() const {
    return _jacobianOplusXj;
  }

  virtual void constructQuadraticForm();

  virtual void mapHessianMemory(double* d, int i, int j, bool rowMajor);

  using BaseEdge<D, E>::resize;
  using BaseEdge<D, E>::computeError;

 protected:
  using BaseEdge<D, E>::_measurement;
  using BaseEdge<D, E>::_information;
  using BaseEdge<D, E>::_error;
  using BaseEdge<D, E>::_vertices;
  using BaseEdge<D, E>::_dimension;

  bool _hessianRowMajor;
  HessianBlockType _hessian;
  HessianBlockTransposedType _hessianTransposed;
  JacobianXiOplusType _jacobianOplusXi;
  JacobianXjOplusType _jacobianOplusXj;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#include "base_binary_edge.hpp"

G2O_END_NAMESPACE

#endif
