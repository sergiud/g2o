// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#include "graph_optimizer_sparse_incremental.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>

#include <memory>
#include <utility>

#include "g2o/examples/interactive_slam/g2o_interactive/types_slam2d_online.h"
#include "g2o/examples/interactive_slam/g2o_interactive/types_slam3d_online.h"

#define DIM_TO_SOLVER(p, l) BlockSolver<BlockSolverTraits<p, l>>

#define ALLOC_CHOLMOD(s, p, l)                                                 \
  if (1) {                                                                     \
    ngx::Log(                                                                  \
        "# Using CHOLMOD online poseDim {} landMarkDim {} blockordering 1", p, \
        l);                                                                    \
    auto linearSolver = std::make_unique<                                      \
        LinearSolverCholmodOnline<DIM_TO_SOLVER(p, l)::PoseMatrixType>>();     \
    s = std::make_unique<DIM_TO_SOLVER(p, l)>(std::move(linearSolver));        \
  } else                                                                       \
    (void)0

using namespace std;
using namespace Eigen;

G2O_START_NAMESPACE

namespace {
/**
 * \brief backing up some information about the vertex
 */
struct VertexBackup {
  int hessianIndex;
  OptimizableGraph::Vertex* vertex;
  double* hessianData;
  bool operator<(const VertexBackup& other) const {
    return hessianIndex < other.hessianIndex;
  }
};
}  // namespace

SparseOptimizerIncremental::SparseOptimizerIncremental() {
  _cholmodSparse = std::make_unique<CholmodExt>();
  cholmod_start(&_cholmodCommon);

  // setup ordering strategy to not permute the matrix
  _cholmodCommon.nmethods = 1;
  _cholmodCommon.method[0].ordering = CHOLMOD_NATURAL;
  _cholmodCommon.postorder = 0;
  _cholmodCommon.supernodal = CHOLMOD_SIMPLICIAL;

  _permutedUpdate.reset(cholmod_allocate_triplet(
      1000, 1000, 1024, 0, CHOLMOD_REAL, &_cholmodCommon));
  L_ = nullptr;
  _solverInterface = nullptr;

  _permutedUpdateAsSparse = std::make_unique<CholmodExt>();
}

SparseOptimizerIncremental::~SparseOptimizerIncremental() {
  _updateMat.clear(true);
  _cholmodFactor.reset();
  _permutedUpdate.reset();
  cholmod_finish(&_cholmodCommon);
}

int SparseOptimizerIncremental::optimize(int iterations, bool online) {
  (void)iterations;  // we only do one iteration anyhow
  OptimizationAlgorithm* solver = _algorithm.get();
  solver->init(online);

  int cjIterations = 0;
  bool ok = true;

  if (!online || batchStep) {
    // cerr << "performing batch step" << endl;
    if (!online) {
      ok = _underlyingSolver->buildStructure();
      if (!ok) {
        ngx::Log(ngx::Error, "Failure while building CCS structure");
        return 0;
      }
    }

    // copy over the updated estimate as new linearization point
    if (slamDimension == 3) {
      for (auto i : indexMapping()) {
        OnlineVertexSE2* v = dynamic_cast<OnlineVertexSE2*>(i);
        v->setEstimate(v->updatedEstimate);
      }
    } else if (slamDimension == 6) {
      for (auto i : indexMapping()) {
        OnlineVertexSE3* v = dynamic_cast<OnlineVertexSE3*>(i);
        v->setEstimate(v->updatedEstimate);
      }
    }

    SparseOptimizer::computeActiveErrors();
    // SparseOptimizer::linearizeSystem();
    _underlyingSolver->buildSystem();

    // mark vertices to be sorted as last
    int numBlocksRequired = _ivMap.size();
    if (_cmember.size() < numBlocksRequired) {
      _cmember.resize(2 * numBlocksRequired);
    }
    memset(_cmember.data(), 0, numBlocksRequired * sizeof(int));
    if (_ivMap.size() > 100) {
      for (size_t i = _ivMap.size() - 20; i < _ivMap.size(); ++i) {
        const HyperGraph::EdgeSet& eset = _ivMap[i]->edges();
        for (auto it : eset) {
          OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
          OptimizableGraph::Vertex* v1 =
              dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
          OptimizableGraph::Vertex* v2 =
              dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
          if (v1->hessianIndex() >= 0) _cmember(v1->hessianIndex()) = 1;
          if (v2->hessianIndex() >= 0) _cmember(v2->hessianIndex()) = 1;
        }
      }
      // OptimizableGraph::Vertex* lastPose = _ivMap.back();
      //_cmember(lastPose->hessianIndex()) = 2;
    }

    ok = _underlyingSolver->solve();

    // get the current cholesky factor along with the permutation
    L_ = _solverInterface->L();
    if (_perm.size() < (int)L_->n) _perm.resize(2 * L_->n);
    int* p = (int*)L_->Perm;
    for (size_t i = 0; i < L_->n; ++i) _perm[p[i]] = i;

  } else {
    // update the b vector
    for (auto _touchedVertice : _touchedVertices) {
      OptimizableGraph::Vertex* v =
          dynamic_cast<OptimizableGraph::Vertex*>(_touchedVertice);
      int iBase = v->colInHessian();
      v->copyB(_underlyingSolver->b() + iBase);
    }
    _solverInterface->solve(_underlyingSolver->x(), _underlyingSolver->b());
  }

  // print statistics for the non-zeros
  // static ofstream debugNonZeros("non-zeros.txt");
  // debugNonZeros << _solverInterface->nonZerosInL() << endl;

  update(_underlyingSolver->x());
  ++cjIterations;

  if (verbose()) {
    computeActiveErrors();
    ngx::Log("nodes = {}\t edges= {}\t chi2= {:f}", vertices().size(),
             _activeEdges.size(), activeChi2());
  }

  if (vizWithGnuplot) gnuplotVisualization();

  if (!ok) return 0;
  return 1;
}

bool SparseOptimizerIncremental::updateInitialization(
    HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset) {
  if (batchStep) {
    return SparseOptimizerOnline::updateInitialization(vset, eset);
  }

  for (auto it : vset) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
    v->clearQuadraticForm();  // be sure that b is zero for this vertex
  }

  // get the touched vertices
  _touchedVertices.clear();
  for (auto it : eset) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
    OptimizableGraph::Vertex* v1 =
        dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
    OptimizableGraph::Vertex* v2 =
        dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
    if (!v1->fixed()) _touchedVertices.insert(v1);
    if (!v2->fixed()) _touchedVertices.insert(v2);
  }
  // cerr << PVAR(_touchedVertices.size()) << endl;

  // updating the internal structures
  std::vector<HyperGraph::Vertex*> newVertices;
  newVertices.reserve(vset.size());
  _activeVertices.reserve(_activeVertices.size() + vset.size());
  _activeEdges.reserve(_activeEdges.size() + eset.size());
  for (auto it : eset)
    _activeEdges.push_back(dynamic_cast<OptimizableGraph::Edge*>(it));
  // cerr << "updating internal done." << endl;

  // update the index mapping
  size_t next = _ivMap.size();
  for (auto it : vset) {
    OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
    if (!v->fixed()) {
      if (!v->marginalized()) {
        v->setHessianIndex(next);
        _ivMap.push_back(v);
        newVertices.push_back(v);
        _activeVertices.push_back(v);
        next++;
      } else  // not supported right now
        abort();
    } else {
      v->setHessianIndex(-1);
    }
  }
  // cerr << "updating index mapping done." << endl;

  // backup the tempindex and prepare sorting structure
  std::vector<VertexBackup> backupIdx(_touchedVertices.size());
  int idx = 0;
  for (auto _touchedVertice : _touchedVertices) {
    OptimizableGraph::Vertex* v =
        dynamic_cast<OptimizableGraph::Vertex*>(_touchedVertice);
    backupIdx[idx].hessianIndex = v->hessianIndex();
    backupIdx[idx].vertex = v;
    backupIdx[idx].hessianData = v->hessianData();
    ++idx;
  }
  sort(backupIdx.begin(),
       backupIdx.end());  // sort according to the hessianIndex which is the
                          // same order as used later by the optimizer
  for (int i = 0; i < idx; ++i) {
    backupIdx[i].vertex->setHessianIndex(i);
  }
  // cerr << "backup tempindex done." << endl;

  // building the structure of the update
  _updateMat.clear(true);  // get rid of the old matrix structure
  _updateMat.rowBlockIndices().clear();
  _updateMat.colBlockIndices().clear();
  _updateMat.blockCols().clear();

  // placing the current stuff in _updateMat
  MatrixXd* lastBlock = nullptr;
  int sizePoses = 0;
  for (int i = 0; i < idx; ++i) {
    OptimizableGraph::Vertex* v = backupIdx[i].vertex;
    int dim = v->dimension();
    sizePoses += dim;
    _updateMat.rowBlockIndices().push_back(sizePoses);
    _updateMat.colBlockIndices().push_back(sizePoses);
    _updateMat.blockCols().emplace_back();
    int ind = v->hessianIndex();
    // cerr << PVAR(ind) << endl;
    if (ind >= 0) {
      MatrixXd* m = _updateMat.block(ind, ind, true);
      v->mapHessianMemory(m->data());
      lastBlock = m;
    }
  }
  lastBlock->diagonal().array() += 1e-6;  // HACK to get Eigen value > 0

  for (auto it : eset) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
    OptimizableGraph::Vertex* v1 =
        dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
    OptimizableGraph::Vertex* v2 =
        dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);

    int ind1 = v1->hessianIndex();
    if (ind1 == -1) continue;
    int ind2 = v2->hessianIndex();
    if (ind2 == -1) continue;
    bool transposedBlock = ind1 > ind2;
    if (transposedBlock)  // make sure, we allocate the upper triangular block
      swap(ind1, ind2);

    MatrixXd* m = _updateMat.block(ind1, ind2, true);
    e->mapHessianMemory(m->data(), 0, 1, transposedBlock);
  }

  // build the system into _updateMat
  for (auto it : eset) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
    e->computeError();
  }
  for (auto it : eset) {
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
    e->linearizeOplus(jacobianWorkspace());
    e->constructQuadraticForm();
  }

  // restore the original data for the vertex
  for (int i = 0; i < idx; ++i) {
    backupIdx[i].vertex->setHessianIndex(backupIdx[i].hessianIndex);
    if (backupIdx[i].hessianData != nullptr)
      backupIdx[i].vertex->mapHessianMemory(backupIdx[i].hessianData);
  }

  // update the structure of the real block matrix
  bool solverStatus = _algorithm->updateStructure(newVertices, eset);

  bool updateStatus = computeCholeskyUpdate();
  if (!updateStatus) {
    ngx::Log(ngx::Error, "Error while computing update");
  }

  std::unique_ptr<cholmod_sparse, CholmodFree> updateAsSparseFactor(
      cholmod_factor_to_sparse(_cholmodFactor.get(), &_cholmodCommon),
      CholmodFree{&_cholmodCommon});

  // convert CCS update by permuting back to the permutation of L
  if (updateAsSparseFactor->nzmax > _permutedUpdate->nzmax) {
    // cerr << "realloc _permutedUpdate" << endl;
    cholmod_reallocate_triplet(updateAsSparseFactor->nzmax,
                               _permutedUpdate.get(), &_cholmodCommon);
  }
  _permutedUpdate->nnz = 0;
  _permutedUpdate->nrow = _permutedUpdate->ncol = L_->n;
  {
    int* Ap = (int*)updateAsSparseFactor->p;
    int* Ai = (int*)updateAsSparseFactor->i;
    double* Ax = (double*)updateAsSparseFactor->x;
    int* Bj = (int*)_permutedUpdate->j;
    int* Bi = (int*)_permutedUpdate->i;
    double* Bx = (double*)_permutedUpdate->x;
    for (size_t c = 0; c < updateAsSparseFactor->ncol; ++c) {
      const int& rbeg = Ap[c];
      const int& rend = Ap[c + 1];
      int cc = c / slamDimension;
      int coff = c % slamDimension;
      const int& cbase = backupIdx[cc].vertex->colInHessian();
      const int& ccol = _perm(cbase + coff);
      for (int j = rbeg; j < rend; j++) {
        const int& r = Ai[j];
        const double& val = Ax[j];

        int rr = r / slamDimension;
        int roff = r % slamDimension;
        const int& rbase = backupIdx[rr].vertex->colInHessian();

        int row = _perm(rbase + roff);
        int col = ccol;
        if (col > row)  // lower triangular entry
          swap(col, row);
        Bi[_permutedUpdate->nnz] = row;
        Bj[_permutedUpdate->nnz] = col;
        Bx[_permutedUpdate->nnz] = val;
        ++_permutedUpdate->nnz;
      }
    }
  }

#if 0
    std::unique_ptr<cholmod_sparse, CholmodFree> updatePermuted(
      cholmod_triplet_to_sparse(_permutedUpdate.get(), _permutedUpdate->nnz, &_cholmodCommon), CholmodFree{&_cholmodCommon});
    //writeCCSMatrix("update-permuted.txt", updatePermuted->nrow, updatePermuted->ncol, (int*)updatePermuted->p, (int*)updatePermuted->i, (double*)updatePermuted->x, false);
    _solverInterface->choleskyUpdate(updatePermuted.get());
#else
  convertTripletUpdateToSparse();
  _solverInterface->choleskyUpdate(_permutedUpdateAsSparse.get());
#endif

  return solverStatus;
}

bool SparseOptimizerIncremental::computeCholeskyUpdate() {
  _cholmodFactor.reset();

  const SparseBlockMatrix<MatrixXd>& A = _updateMat;
  size_t m = A.rows();
  size_t n = A.cols();

  if (_cholmodSparse->columnsAllocated < n) {
    // std::cerr << __PRETTY_FUNCTION__ << ": reallocating columns" <<
    // std::endl;
    _cholmodSparse->columnsAllocated =
        _cholmodSparse->columnsAllocated == 0
            ? n
            : 2 * n;  // pre-allocate more space if re-allocating
    _cholmodSparse->allocateColumns(_cholmodSparse->columnsAllocated);
  }
  size_t nzmax = A.nonZeros();
  if (_cholmodSparse->nzmax < nzmax) {
    // std::cerr << __PRETTY_FUNCTION__ << ": reallocating row + values" <<
    // std::endl;
    _cholmodSparse->nzmax =
        _cholmodSparse->nzmax == 0
            ? nzmax
            : 2 * nzmax;  // pre-allocate more space if re-allocating
    _cholmodSparse->allocateEntries(_cholmodSparse->nzmax);
  }
  _cholmodSparse->ncol = n;
  _cholmodSparse->nrow = m;

  A.fillCCS((int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
            (double*)_cholmodSparse->x, true);
  // writeCCSMatrix("updatesparse.txt", _cholmodSparse->nrow,
  // _cholmodSparse->ncol, (int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
  // (double*)_cholmodSparse->x, true);

  _cholmodFactor.reset(cholmod_analyze(_cholmodSparse.get(), &_cholmodCommon));
  cholmod_factorize(_cholmodSparse.get(), _cholmodFactor.get(),
                    &_cholmodCommon);

#if 0
    int* p = (int*)_cholmodFactor->Perm;
    for (int i = 0; i < (int)n; ++i)
      if (i != p[i])
        cerr << "wrong permutation" << i << " -> " << p[i] << endl;
#endif

  if (_cholmodCommon.status == CHOLMOD_NOT_POSDEF) {
    // std::cerr << "Cholesky failure, writing debug.txt (Hessian loadable by
    // Octave)" << std::endl; writeCCSMatrix("debug.txt", _cholmodSparse->nrow,
    // _cholmodSparse->ncol, (int*)_cholmodSparse->p, (int*)_cholmodSparse->i,
    // (double*)_cholmodSparse->x, true);
    return false;
  }

  // change to the specific format we need to have a pretty normal L
  int change_status = cholmod_change_factor(
      CHOLMOD_REAL, 1, 0, 1, 1, _cholmodFactor.get(), &_cholmodCommon);
  if (change_status == 0) {
    return false;
  }

  return true;
}

static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& solverName) {
  std::unique_ptr<Solver> s;

  if (solverName == "fix3_2_cholmod") {
    ALLOC_CHOLMOD(s, 3, 2);
  } else if (solverName == "fix6_3_cholmod") {
    ALLOC_CHOLMOD(s, 6, 3);
  }

  return std::make_unique<OptimizationAlgorithmGaussNewton>(std::move(s));
}

bool SparseOptimizerIncremental::initSolver(int dimension, int batchEveryN) {
  // cerr << __PRETTY_FUNCTION__ << endl;
  slamDimension = dimension;
  if (dimension == 3) {
    setAlgorithm(createSolver("fix3_2_cholmod"));
    OptimizationAlgorithmGaussNewton* gaussNewton =
        dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver());
    assert(gaussNewton);
    BlockSolver<BlockSolverTraits<3, 2>>* bs =
        dynamic_cast<BlockSolver<BlockSolverTraits<3, 2>>*>(
            gaussNewton->solver());
    assert(bs && "Unable to get internal block solver");
    LinearSolverCholmodOnline<Matrix3d>* s =
        dynamic_cast<LinearSolverCholmodOnline<Matrix3d>*>(bs->linearSolver());
    bs->setAdditionalVectorSpace(300);
    bs->setSchur(false);
    _solverInterface = s;
    _underlyingSolver = bs;
  } else {
    setAlgorithm(createSolver("fix6_3_cholmod"));
    OptimizationAlgorithmGaussNewton* gaussNewton =
        dynamic_cast<OptimizationAlgorithmGaussNewton*>(solver());
    assert(gaussNewton);
    BlockSolver<BlockSolverTraits<6, 3>>* bs =
        dynamic_cast<BlockSolver<BlockSolverTraits<6, 3>>*>(
            gaussNewton->solver());
    assert(bs && "Unable to get internal block solver");
    LinearSolverCholmodOnline<Matrix<double, 6, 6>>* s =
        dynamic_cast<LinearSolverCholmodOnline<Matrix<double, 6, 6>>*>(
            bs->linearSolver());
    bs->setAdditionalVectorSpace(600);
    bs->setSchur(false);
    _solverInterface = s;
    _underlyingSolver = bs;
  }
  _solverInterface->cmember = &_cmember;
  _solverInterface->batchEveryN = batchEveryN;
  if (solver() == nullptr) {
    ngx::Log(ngx::Error,
             "Error allocating solver. Allocating CHOLMOD solver failed!");
    return false;
  }
  return true;
}

void SparseOptimizerIncremental::convertTripletUpdateToSparse() {
  // re-allocate the memory
  if (_tripletWorkspace.size() < (int)_permutedUpdate->ncol) {
    _tripletWorkspace.resize(_permutedUpdate->ncol * 2);
  }

  // reallocate num-zeros
  if (_permutedUpdateAsSparse->nzmax < _permutedUpdate->nzmax) {
    _permutedUpdateAsSparse->nzmax = _permutedUpdate->nzmax;
    _permutedUpdateAsSparse->allocateEntries(_permutedUpdateAsSparse->nzmax);
  }

  if (_permutedUpdateAsSparse->columnsAllocated < _permutedUpdate->ncol) {
    _permutedUpdateAsSparse->columnsAllocated = 2 * _permutedUpdate->ncol;
    _permutedUpdateAsSparse->allocateColumns(
        _permutedUpdateAsSparse->columnsAllocated);
  }

  _permutedUpdateAsSparse->ncol = _permutedUpdate->ncol;
  _permutedUpdateAsSparse->nrow = _permutedUpdate->nrow;

  int* w = _tripletWorkspace.data();
  memset(w, 0, sizeof(int) * _permutedUpdate->ncol);

  int* Ti = (int*)_permutedUpdate->i;
  int* Tj = (int*)_permutedUpdate->j;
  double* Tx = (double*)_permutedUpdate->x;

  int* Cp = (int*)_permutedUpdateAsSparse->p;
  int* Ci = (int*)_permutedUpdateAsSparse->i;
  double* Cx = (double*)_permutedUpdateAsSparse->x;

  for (size_t k = 0; k < _permutedUpdate->nnz; ++k) /* column counts */
    w[Tj[k]]++;

  /* column pointers */
  int n = _permutedUpdate->ncol;
  int nz = 0;
  for (int i = 0; i < n; i++) {
    Cp[i] = nz;
    nz += w[i];
    w[i] = Cp[i];
  }
  Cp[n] = nz;
  assert((size_t)nz == _permutedUpdate->nnz);

  int p;
  for (size_t k = 0; k < _permutedUpdate->nnz; ++k) {
    p = w[Tj[k]]++;
    Ci[p] = Ti[k]; /* A(i,j) is the pth entry in C */
    Cx[p] = Tx[k];
  }
}

G2O_END_NAMESPACE
