// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_CORE_MATRIX_OPERATIONS_H
#define G2O_CORE_MATRIX_OPERATIONS_H

#include <g2o/core/abi.h>

#include <Eigen/Core>

G2O_START_NAMESPACE
namespace internal {

template <typename MatrixType>
inline void axpy(const MatrixType& A,
                 const Eigen::Map<const Eigen::VectorXd>& x, int xoff,
                 Eigen::Map<Eigen::VectorXd>& y, int yoff) {
  y.segment<MatrixType::RowsAtCompileTime>(yoff) +=
      A * x.segment<MatrixType::ColsAtCompileTime>(xoff);
}

template <int t>
inline void axpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A,
                 const Eigen::Map<const Eigen::VectorXd>& x, int xoff,
                 Eigen::Map<Eigen::VectorXd>& y, int yoff) {
  y.segment(yoff, A.rows()) +=
      A *
      x.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(
          xoff);
}

template <>
inline void axpy<Eigen::MatrixXd>(const Eigen::MatrixXd& A,
                                  const Eigen::Map<const Eigen::VectorXd>& x,
                                  int xoff, Eigen::Map<Eigen::VectorXd>& y,
                                  int yoff) {
  y.segment(yoff, A.rows()) += A * x.segment(xoff, A.cols());
}

template <typename MatrixType>
inline void atxpy(const MatrixType& A,
                  const Eigen::Map<const Eigen::VectorXd>& x, int xoff,
                  Eigen::Map<Eigen::VectorXd>& y, int yoff) {
  y.segment<MatrixType::ColsAtCompileTime>(yoff) +=
      A.transpose() * x.segment<MatrixType::RowsAtCompileTime>(xoff);
}

template <int t>
inline void atxpy(const Eigen::Matrix<double, Eigen::Dynamic, t>& A,
                  const Eigen::Map<const Eigen::VectorXd>& x, int xoff,
                  Eigen::Map<Eigen::VectorXd>& y, int yoff) {
  y.segment<Eigen::Matrix<double, Eigen::Dynamic, t>::ColsAtCompileTime>(
      yoff) += A.transpose() * x.segment(xoff, A.rows());
}

template <>
inline void atxpy<Eigen::MatrixXd>(const Eigen::MatrixXd& A,
                                   const Eigen::Map<const Eigen::VectorXd>& x,
                                   int xoff, Eigen::Map<Eigen::VectorXd>& y,
                                   int yoff) {
  y.segment(yoff, A.cols()) += A.transpose() * x.segment(xoff, A.rows());
}

}  // end namespace internal
G2O_END_NAMESPACE

#endif
