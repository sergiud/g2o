// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

namespace internal {
  inline int computeUpperTriangleIndex(int i, int j)
  {
    int elemsUpToCol = ((j-1) * j) / 2;
    return elemsUpToCol + i;
  }
} // namespace internal

template <int D, typename E>
void BaseMultiEdge<D, E>::constructQuadraticForm()
{
  if (this->robustKernel()) {
    double error = this->chi2();
    Vector3D rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<double, D, 1, Eigen::ColMajor> omega_r = - _information * _error;
    omega_r *= rho[1];
    computeQuadraticForm(this->robustInformation(rho), omega_r);
  } else {
    computeQuadraticForm(_information, - _information * _error);
  }
}


template <int D, typename E>
void BaseMultiEdge<D, E>::linearizeOplus(JacobianWorkspace& jacobianWorkspace)
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    assert(v->dimension() >= 0);
    new (&_jacobianOplus[i]) JacobianType(jacobianWorkspace.workspaceForVertex(i), D, v->dimension());
  }
  linearizeOplus();
}

template <int D, typename E>
void BaseMultiEdge<D, E>::linearizeOplus()
{
#ifdef G2O_OPENMP
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->lockQuadraticForm();
  }
#endif

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  for (size_t i = 0; i < _vertices.size(); ++i) {
    //Xi - estimate the jacobian numerically
    OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);

    if (vi->fixed())
      continue;

    const int vi_dim = vi->dimension();
    assert(vi_dim >= 0);

    std::vector<double> add_vi(static_cast<std::size_t>(vi_dim), 0.0);
    assert(_dimension >= 0);
    assert(_jacobianOplus[i].rows() == _dimension && _jacobianOplus[i].cols() == vi_dim && "jacobian cache dimension does not match");
      _jacobianOplus[i].resize(_dimension, vi_dim);
    // add small step along the unit vector in each dimension
    for (std::size_t d = 0; d < add_vi.size(); ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi.data());
      computeError();
      errorBak = _error;
      vi->pop();
      vi->push();
      add_vi[d] = -delta;
      vi->oplus(add_vi.data());
      computeError();
      errorBak -= _error;
      vi->pop();
      add_vi[d] = 0.0;

      _jacobianOplus[i].col(d) = scalar * errorBak;
    } // end dimension
  }
  _error = errorBeforeNumeric;

#ifdef G2O_OPENMP
  for (int i = (int)(_vertices.size()) - 1; i >= 0; --i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->unlockQuadraticForm();
  }
#endif

}

template <int D, typename E>
void BaseMultiEdge<D, E>::mapHessianMemory(double* d, int i, int j, bool rowMajor)
{
  int idx = internal::computeUpperTriangleIndex(i, j);
  assert(idx < (int)_hessian.size());
  OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(i));
  OptimizableGraph::Vertex* vj = static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(j));
  assert(vi->dimension() >= 0);
  assert(vj->dimension() >= 0);
  HessianHelper& h = _hessian[idx];
  if (rowMajor) {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vj->dimension(), vi->dimension());
  } else {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vi->dimension(), vj->dimension());
  }
  h.transposed = rowMajor;
}

template <int D, typename E>
void BaseMultiEdge<D, E>::resize(size_t size)
{
  BaseEdge<D,E>::resize(size);
  int n = (int)_vertices.size();
  int maxIdx = (n * (n-1))/2;
  assert(maxIdx >= 0);
  _hessian.resize(maxIdx);
  _jacobianOplus.resize(size, JacobianType(nullptr,0,0));
}

template <int D, typename E>
bool BaseMultiEdge<D, E>::allVerticesFixed() const
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    if (!static_cast<const OptimizableGraph::Vertex*> (_vertices[i])->fixed()) {
      return false;
    }
  }
  return true;
}

template <int D, typename E>
void BaseMultiEdge<D, E>::computeQuadraticForm(const InformationType& omega, const ErrorVector& weightedError)
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    bool istatus = !(from->fixed());

    if (istatus) {
      const JacobianType& A = _jacobianOplus[i];

      MatrixXD AtO = A.transpose() * omega;
      int fromDim = from->dimension();
      assert(fromDim >= 0);
      Eigen::Map<MatrixXD> fromMap(from->hessianData(), fromDim, fromDim);
      Eigen::Map<VectorXD> fromB(from->bData(), fromDim);

      // ii block in the hessian
#ifdef G2O_OPENMP
      from->lockQuadraticForm();
#endif
      fromMap.noalias() += AtO * A;
      fromB.noalias() += A.transpose() * weightedError;

      // compute the off-diagonal blocks ij for all j
      for (size_t j = i+1; j < _vertices.size(); ++j) {
        OptimizableGraph::Vertex* to = static_cast<OptimizableGraph::Vertex*>(_vertices[j]);
#ifdef G2O_OPENMP
        to->lockQuadraticForm();
#endif
        bool jstatus = !(to->fixed());
        if (jstatus) {
          const JacobianType& B = _jacobianOplus[j];
          int idx = internal::computeUpperTriangleIndex(i, j);
          assert(idx < (int)_hessian.size());
          HessianHelper& hhelper = _hessian[idx];
          if (hhelper.transposed) { // we have to write to the block as transposed
            hhelper.matrix.noalias() += B.transpose() * AtO.transpose();
          } else {
            hhelper.matrix.noalias() += AtO * B;
          }
        }
#ifdef G2O_OPENMP
        to->unlockQuadraticForm();
#endif
      }

#ifdef G2O_OPENMP
      from->unlockQuadraticForm();
#endif
    }

  }
}


// PARTIAL TEMPLATE SPECIALIZATION

template <typename E>
void BaseMultiEdge<-1, E>::constructQuadraticForm()
{
  if (this->robustKernel()) {
    double error = this->chi2();
    Vector3D rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::ColMajor> omega_r = - _information * _error;
    omega_r *= rho[1];
    computeQuadraticForm(this->robustInformation(rho), omega_r);
  } else {
    computeQuadraticForm(_information, - _information * _error);
  }
}


template <typename E>
void BaseMultiEdge<-1, E>::linearizeOplus(JacobianWorkspace& jacobianWorkspace)
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    assert(v->dimension() >= 0);
    new (&_jacobianOplus[i]) JacobianType(jacobianWorkspace.workspaceForVertex(i), _dimension, v->dimension());
  }
  linearizeOplus();
}

template <typename E>
void BaseMultiEdge<-1, E>::linearizeOplus()
{
#ifdef G2O_OPENMP
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->lockQuadraticForm();
  }
#endif

  const double delta = 1e-9;
  const double scalar = 1.0 / (2*delta);
  ErrorVector errorBak;
  ErrorVector errorBeforeNumeric = _error;

  for (size_t i = 0; i < _vertices.size(); ++i) {
    //Xi - estimate the jacobian numerically
    OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);

    if (vi->fixed())
      continue;

    const int vi_dim = vi->dimension();
    assert(vi_dim >= 0);

    std::vector<double> add_vi(static_cast<std::size_t>(vi_dim), 0.0);
    assert(_dimension >= 0);
    assert(_jacobianOplus[i].rows() == _dimension && _jacobianOplus[i].cols() == vi_dim && "jacobian cache dimension does not match");
    _jacobianOplus[i].resize(_dimension, vi_dim);
    // add small step along the unit vector in each dimension
    for (std::size_t d = 0; d < add_vi.size(); ++d) {
      vi->push();
      add_vi[d] = delta;
      vi->oplus(add_vi.data());
      computeError();
      errorBak = _error;
      vi->pop();
      vi->push();
      add_vi[d] = -delta;
      vi->oplus(add_vi.data());
      computeError();
      errorBak -= _error;
      vi->pop();
      add_vi[d] = 0.0;

      _jacobianOplus[i].col(d) = scalar * errorBak;
    } // end dimension
  }
  _error = errorBeforeNumeric;

#ifdef G2O_OPENMP
  for (int i = (int)(_vertices.size()) - 1; i >= 0; --i) {
    OptimizableGraph::Vertex* v = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    v->unlockQuadraticForm();
  }
#endif

}

template <typename E>
void BaseMultiEdge<-1, E>::mapHessianMemory(double* d, int i, int j, bool rowMajor)
{
  int idx = internal::computeUpperTriangleIndex(i, j);
  assert(idx < (int)_hessian.size());
  OptimizableGraph::Vertex* vi = static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(i));
  OptimizableGraph::Vertex* vj = static_cast<OptimizableGraph::Vertex*>(HyperGraph::Edge::vertex(j));
  assert(vi->dimension() >= 0);
  assert(vj->dimension() >= 0);
  HessianHelper& h = _hessian[idx];
  if (rowMajor) {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vj->dimension(), vi->dimension());
  } else {
    if (h.matrix.data() != d || h.transposed != rowMajor)
      new (&h.matrix) HessianBlockType(d, vi->dimension(), vj->dimension());
  }
  h.transposed = rowMajor;
}

template <typename E>
void BaseMultiEdge<-1, E>::resize(size_t size)
{
  BaseEdge<-1,E>::resize(size);
  int n = (int)_vertices.size();
  int maxIdx = (n * (n-1))/2;
  assert(maxIdx >= 0);
  _hessian.resize(maxIdx);
  _jacobianOplus.resize(size, JacobianType(nullptr,0,0));
}

template <typename E>
bool BaseMultiEdge<-1, E>::allVerticesFixed() const
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    if (!static_cast<const OptimizableGraph::Vertex*> (_vertices[i])->fixed()) {
      return false;
    }
  }
  return true;
}

template <typename E>
void BaseMultiEdge<-1, E>::computeQuadraticForm(const InformationType& omega, const ErrorVector& weightedError)
{
  for (size_t i = 0; i < _vertices.size(); ++i) {
    OptimizableGraph::Vertex* from = static_cast<OptimizableGraph::Vertex*>(_vertices[i]);
    bool istatus = !(from->fixed());

    if (istatus) {
      const JacobianType& A = _jacobianOplus[i];

      MatrixXD AtO = A.transpose() * omega;
      int fromDim = from->dimension();
      assert(fromDim >= 0);
      Eigen::Map<MatrixXD> fromMap(from->hessianData(), fromDim, fromDim);
      Eigen::Map<VectorXD> fromB(from->bData(), fromDim);

      // ii block in the hessian
#ifdef G2O_OPENMP
      from->lockQuadraticForm();
#endif
      fromMap.noalias() += AtO * A;
      fromB.noalias() += A.transpose() * weightedError;

      // compute the off-diagonal blocks ij for all j
      for (size_t j = i+1; j < _vertices.size(); ++j) {
        OptimizableGraph::Vertex* to = static_cast<OptimizableGraph::Vertex*>(_vertices[j]);
#ifdef G2O_OPENMP
        to->lockQuadraticForm();
#endif
        bool jstatus = !(to->fixed());
        if (jstatus) {
          const JacobianType& B = _jacobianOplus[j];
          int idx = internal::computeUpperTriangleIndex(i, j);
          assert(idx < (int)_hessian.size());
          HessianHelper& hhelper = _hessian[idx];
          if (hhelper.transposed) { // we have to write to the block as transposed
            hhelper.matrix.noalias() += B.transpose() * AtO.transpose();
          } else {
            hhelper.matrix.noalias() += AtO * B;
          }
        }
#ifdef G2O_OPENMP
        to->unlockQuadraticForm();
#endif
      }

#ifdef G2O_OPENMP
      from->unlockQuadraticForm();
#endif
    }

  }
}
