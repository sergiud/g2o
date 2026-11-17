// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

template <int D, typename T>
BaseVertex<D, T>::BaseVertex()
    : OptimizableGraph::Vertex(), _hessian(nullptr, D, D) {
  _dimension = D;
}

template <int D, typename T>
double BaseVertex<D, T>::solveDirect(double lambda) {
  Eigen::Matrix<double, D, D, Eigen::ColMajor> tempA =
      _hessian +
      Eigen::Matrix<double, D, D, Eigen::ColMajor>::Identity() * lambda;
  double det = tempA.determinant();
  if (g2o_isnan(det) || det < std::numeric_limits<double>::epsilon())
    return det;
  Eigen::Matrix<double, D, 1, Eigen::ColMajor> dx = tempA.llt().solve(_b);
  oplus(&dx[0]);
  return det;
}

template <int D, typename T>
void BaseVertex<D, T>::clearQuadraticForm() {
  _b.setZero();
}

template <int D, typename T>
void BaseVertex<D, T>::mapHessianMemory(double* d) {
  new (&_hessian) HessianBlockType(d, D, D);
}
