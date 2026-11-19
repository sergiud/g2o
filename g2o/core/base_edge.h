// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_BASE_EDGE_H
#define G2O_BASE_EDGE_H

#include <g2o/config.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/prefix.h>
#include <g2o/stuff/logger.h>

#include <Eigen/Core>
#include <limits>

G2O_START_NAMESPACE

template <int D, typename E>
class BaseEdge : public OptimizableGraph::Edge {
 public:
  static const int Dimension = D;
  typedef E Measurement;
  typedef Eigen::Matrix<double, D, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<double, D, D, Eigen::ColMajor> InformationType;

  BaseEdge() { _dimension = D; }

  virtual double chi2() const { return _error.dot(information() * _error); }

  virtual const double* errorData() const { return _error.data(); }
  virtual double* errorData() { return _error.data(); }
  const ErrorVector& error() const { return _error; }
  ErrorVector& error() { return _error; }

  //! information matrix of the constraint
  EIGEN_STRONG_INLINE const InformationType& information() const {
    return _information;
  }
  EIGEN_STRONG_INLINE InformationType& information() { return _information; }
  EIGEN_STRONG_INLINE void setInformation(const InformationType& information) {
    _information = information;
  }

  virtual const double* informationData() const { return _information.data(); }
  virtual double* informationData() { return _information.data(); }

  //! accessor functions for the measurement represented by the edge
  EIGEN_STRONG_INLINE const Measurement& measurement() const {
    return _measurement;
  }
  virtual void setMeasurement(const Measurement& m) { _measurement = m; }

  virtual int rank() const { return _dimension; }

  virtual void initialEstimate(const OptimizableGraph::VertexSet&,
                               OptimizableGraph::Vertex*) {
    ngx::Log(ngx::Error,
             "inititialEstimate() is not implemented, please give "
             "implementation in your derived class");
  }

 protected:
  Measurement _measurement;
  InformationType _information;
  ErrorVector _error;

  /**
   * calculate the robust information matrix by updating the information matrix
   * of the error
   */
  InformationType robustInformation(const Vector3D& rho) {
    InformationType result = rho[1] * _information;
    // ErrorVector weightedErrror = _information * _error;
    // result.noalias() += 2 * rho[2] * (weightedErrror *
    // weightedErrror.transpose());
    return result;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template <typename E>
class BaseEdge<-1, E> : public OptimizableGraph::Edge {
 public:
  static const int Dimension = -1;
  typedef E Measurement;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> ErrorVector;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>
      InformationType;

  BaseEdge() = default;

  virtual double chi2() const { return _error.dot(information() * _error); }

  virtual const double* errorData() const { return _error.data(); }
  virtual double* errorData() { return _error.data(); }
  const ErrorVector& error() const { return _error; }
  ErrorVector& error() { return _error; }

  //! information matrix of the constraint
  const InformationType& information() const { return _information; }
  InformationType& information() { return _information; }
  void setInformation(const InformationType& information) {
    _information = information;
  }

  virtual const double* informationData() const { return _information.data(); }
  virtual double* informationData() { return _information.data(); }

  //! accessor functions for the measurement represented by the edge
  const Measurement& measurement() const { return _measurement; }
  virtual void setMeasurement(const Measurement& m) { _measurement = m; }

  virtual int rank() const { return _dimension; }

  virtual void initialEstimate(const OptimizableGraph::VertexSet&,
                               OptimizableGraph::Vertex*) {
    ngx::Log(ngx::Error,
             "inititialEstimate() is not implemented, please give "
             "implementation in your derived class");
  }

 protected:
  Measurement _measurement;
  InformationType _information;
  ErrorVector _error;

  /**
   * calculate the robust information matrix by updating the information matrix
   * of the error
   */
  InformationType robustInformation(const Vector3D& rho) {
    InformationType result = rho[1] * _information;
    // ErrorVector weightedErrror = _information * _error;
    // result.noalias() += 2 * rho[2] * (weightedErrror *
    // weightedErrror.transpose());
    return result;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#ifndef core_EXPORTS
G2O_EXTERN_TEMPLATE class G2O_CORE_API BaseEdge<1, double>;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<1, Eigen::Matrix<double, 1, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<2, Eigen::Matrix<double, 2, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<3, Eigen::Matrix<double, 3, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<4, Eigen::Matrix<double, 4, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<5, Eigen::Matrix<double, 5, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<6, Eigen::Matrix<double, 6, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API
BaseEdge<7, Eigen::Matrix<double, 7, 1> >;
G2O_EXTERN_TEMPLATE class G2O_CORE_API BaseEdge<6, Eigen::Isometry3d>;
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
