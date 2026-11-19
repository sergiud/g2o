// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/logger.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o_interactive/g2o_slam_interface.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <utility>

#include "fast_output.h"
#include "graph_optimizer_sparse_online.h"
#include "types_slam2d_online.h"
#include "types_slam3d_online.h"
using namespace std;
using namespace Eigen;

G2O_START_NAMESPACE

G2oSlamInterface::G2oSlamInterface(SparseOptimizerOnline* optimizer)
    : _optimizer(optimizer),
      _firstOptimization(true),
      _nodesAdded(0),
      _incIterations(1),
      _updateGraphEachN(10),
      _batchEveryN(100),
      _lastBatchStep(0),
      _initSolverDone(false) {}

bool G2oSlamInterface::addNode(const std::string& tag, int id, int dimension,
                               const std::vector<double>& values) {
  // we add the node when we are asked to add the according edge
  (void)tag;
  (void)id;
  (void)values;
  return initSolver(dimension);
}

bool G2oSlamInterface::initSolver(int dimension) {
  constexpr int PlanarDimension = 3;
  constexpr int SpatialDimension = 6;
  if (dimension != PlanarDimension && dimension != SpatialDimension) {
    ngx::Log(ngx::Error, "pose of dimension {} rejected, expected {} or {}",
             dimension, PlanarDimension, SpatialDimension);
    return false;
  }
  // The first pose, declared or introduced by an edge, decides the solver.
  if (!_initSolverDone) {
    _initSolverDone = true;
    if (!_optimizer->initSolver(dimension, _batchEveryN)) {
      return false;
    }
  }
  if (dimension != _optimizer->slamDimension) {
    ngx::Log(ngx::Error,
             "pose of dimension {} rejected by the solver for dimension {}",
             dimension, _optimizer->slamDimension);
    return false;
  }
  return true;
}

bool G2oSlamInterface::addEdge(const std::string& tag, int id, int dimension,
                               int v1Id, int v2Id,
                               const std::vector<double>& measurement,
                               const std::vector<double>& information) {
  (void)tag;
  (void)id;
  size_t oldEdgesSize = _optimizer->edges().size();

  if (!initSolver(dimension)) {
    return false;
  }

  if (v1Id == v2Id) {
    ngx::Log(ngx::Error, "edge from vertex {} to itself rejected", v1Id);
    return false;
  }

  if (dimension == 3) {
    SE2 transf(measurement[0], measurement[1], measurement[2]);
    Eigen::Matrix3d infMat;
    int idx = 0;
    for (int r = 0; r < 3; ++r)
      for (int c = r; c < 3; ++c, ++idx) {
        assert(idx < (int)information.size());
        infMat(r, c) = infMat(c, r) = information[idx];
      }
    // cerr << PVAR(infMat) << endl;

    int doInit = 0;
    SparseOptimizer::Vertex* v1 = _optimizer->vertex(v1Id);
    SparseOptimizer::Vertex* v2 = _optimizer->vertex(v2Id);
    if (v1 == nullptr) {
      OptimizableGraph::Vertex* v = v1 = addVertex(dimension, v1Id);
      if (v == nullptr) {
        return false;
      }
      _verticesAdded.insert(v);
      doInit = 1;
      ++_nodesAdded;
    }

    if (v2 == nullptr) {
      OptimizableGraph::Vertex* v = v2 = addVertex(dimension, v2Id);
      if (v == nullptr) {
        return false;
      }
      _verticesAdded.insert(v);
      doInit = 2;
      ++_nodesAdded;
    }

    if (_optimizer->edges().empty()) {
      if (v1->id() < v2->id()) {
        ngx::Log("FIRST EDGE fixing {}", v1->id());
        v1->setFixed(true);
      } else {
        ngx::Log("FIRST EDGE fixing {}", v2->id());
        v2->setFixed(true);
      }
    }

    auto eOwner = std::make_unique<OnlineEdgeSE2>();
    auto* e = eOwner.get();
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    _optimizer->addEdge(std::move(eOwner));
    _edgesAdded.insert(e);

    if (doInit != 0) {
      OptimizableGraph::Vertex* from =
          dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
      OptimizableGraph::Vertex* to =
          dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
      switch (doInit) {
        case 1:  // initialize v1 from v2
        {
          HyperGraph::VertexSet toSet;
          toSet.insert(to);
          if (e->initialEstimatePossible(toSet, from) > 0.) {
            e->initialEstimate(toSet, from);
          }
          break;
        }
        case 2: {
          HyperGraph::VertexSet fromSet;
          fromSet.insert(from);
          if (e->initialEstimatePossible(fromSet, to) > 0.) {
            e->initialEstimate(fromSet, to);
          }
          break;
        }
        default:
          ngx::Log(ngx::Error, "doInit wrong value");
      }
    }

  } else if (dimension == 6) {
    Eigen::Isometry3d transf;
    Matrix<double, 6, 6> infMat;

    if (measurement.size() == 7) {  // measurement is a Quaternion
      Vector7d meas;
      for (int i = 0; i < 7; ++i) meas(i) = measurement[i];
      // normalize the quaternion to recover numerical precision lost by storing
      // as human readable text
      Vector4d::MapType(meas.data() + 3).normalize();
      transf = internal::fromVectorQT(meas);

      for (int i = 0, idx = 0; i < infMat.rows(); ++i)
        for (int j = i; j < infMat.cols(); ++j) {
          infMat(i, j) = information[idx++];
          if (i != j) infMat(j, i) = infMat(i, j);
        }
    } else {  // measurement consists of Euler angles
      Vector6d aux;
      aux << measurement[0], measurement[1], measurement[2], measurement[3],
          measurement[4], measurement[5];
      transf = internal::fromVectorET(aux);
      Matrix<double, 6, 6> infMatEuler;
      int idx = 0;
      for (int r = 0; r < 6; ++r)
        for (int c = r; c < 6; ++c, ++idx) {
          assert(idx < (int)information.size());
          infMatEuler(r, c) = infMatEuler(c, r) = information[idx];
        }
      // convert information matrix to our internal representation
      const Matrix<double, 6, 6> J =
          internal::jacobianVectorETByVectorMQT(transf);
      infMat.noalias() = J.transpose() * infMatEuler * J;
      // cerr << PVAR(transf.matrix()) << endl;
      // cerr << PVAR(infMat) << endl;
    }

    int doInit = 0;
    SparseOptimizer::Vertex* v1 = _optimizer->vertex(v1Id);
    SparseOptimizer::Vertex* v2 = _optimizer->vertex(v2Id);
    if (v1 == nullptr) {
      OptimizableGraph::Vertex* v = v1 = addVertex(dimension, v1Id);
      if (v == nullptr) {
        return false;
      }
      _verticesAdded.insert(v);
      doInit = 1;
      ++_nodesAdded;
    }

    if (v2 == nullptr) {
      OptimizableGraph::Vertex* v = v2 = addVertex(dimension, v2Id);
      if (v == nullptr) {
        return false;
      }
      _verticesAdded.insert(v);
      doInit = 2;
      ++_nodesAdded;
    }

    if (_optimizer->edges().empty()) {
      if (v1->id() < v2->id()) {
        ngx::Log("FIRST EDGE fixing {}", v1->id());
        v1->setFixed(true);
      } else {
        ngx::Log("FIRST EDGE fixing {}", v2->id());
        v2->setFixed(true);
      }
    }

    auto eOwner = std::make_unique<OnlineEdgeSE3>();
    auto* e = eOwner.get();
    e->vertices()[0] = v1;
    e->vertices()[1] = v2;
    e->setMeasurement(transf);
    e->setInformation(infMat);
    _optimizer->addEdge(std::move(eOwner));
    _edgesAdded.insert(e);

    if (doInit != 0) {
      OptimizableGraph::Vertex* from =
          dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[0]);
      OptimizableGraph::Vertex* to =
          dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[1]);
      switch (doInit) {
        case 1:  // initialize v1 from v2
        {
          HyperGraph::VertexSet toSet;
          toSet.insert(to);
          if (e->initialEstimatePossible(toSet, from) > 0.) {
            e->initialEstimate(toSet, from);
          }
          break;
        }
        case 2: {
          HyperGraph::VertexSet fromSet;
          fromSet.insert(from);
          if (e->initialEstimatePossible(fromSet, to) > 0.) {
            e->initialEstimate(fromSet, to);
          }
          break;
        }
        default:
          ngx::Log(ngx::Error, "doInit wrong value");
      }
    }

  } else {
    ngx::Log(ngx::Error, "not implemented for this dimension");
    return false;
  }

  if (oldEdgesSize == 0) {
    _optimizer->jacobianWorkspace().allocate();
  }

  return true;
}

bool G2oSlamInterface::fixNode(const std::vector<int>& nodes) {
  for (int node : nodes) {
    OptimizableGraph::Vertex* v = _optimizer->vertex(node);
    if (v != nullptr) v->setFixed(true);
  }
  return true;
}

bool G2oSlamInterface::queryState(const std::vector<int>& nodes) {
  // return true;
  cout << "BEGIN" << endl;
#if 1
  if (nodes.empty()) {
    // The protocol lists the poses by their ids, which the map of the
    // vertices does not order.
    std::vector<int> ids;
    ids.reserve(_optimizer->vertices().size());
    for (const auto& [id, vertex] : _optimizer->vertices()) {
      ids.push_back(id);
    }
    std::ranges::sort(ids);
    for (int id : ids) {
      printVertex(_optimizer->vertex(id));
    }
  } else {
    for (int node : nodes) {
      OptimizableGraph::Vertex* v = _optimizer->vertex(node);
      if (v != nullptr) printVertex(v);
    }
  }
#endif
  cout << "END" << endl << flush;

  return true;
}

bool G2oSlamInterface::solveState() {
  SolveResult state = solve();
  return state != ERROR;
}

OptimizableGraph::Vertex* G2oSlamInterface::addVertex(int dimension, int id) {
  std::unique_ptr<OptimizableGraph::Vertex> vOwner;
  if (dimension == 3) {
    vOwner = std::make_unique<OnlineVertexSE2>();
  } else if (dimension == 6) {
    vOwner = std::make_unique<OnlineVertexSE3>();
  } else {
    return nullptr;
  }
  auto* v = vOwner.get();
  v->setId(id);  // estimate will be set later when the edge is added
  // The graph destroys a vertex it rejects, such as one with a negative id.
  if (!_optimizer->addVertex(std::move(vOwner))) {
    return nullptr;
  }
  return v;
}

bool G2oSlamInterface::printVertex(OptimizableGraph::Vertex* v) {
  static char buffer[10000];  // that should be more than enough
  int vdim = v->dimension();
  if (vdim == 3) {
    char* s = buffer;
    OnlineVertexSE2* v2 = dynamic_cast<OnlineVertexSE2*>(v);
    memcpy(s, "VERTEX_XYT ", 11);
    s += 11;
    s += modp_itoa10(v->id(), s);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.translation().x(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.translation().y(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v2->updatedEstimate.rotation().angle(), s, 6);
    *s++ = '\n';
    cout.write(buffer, s - buffer);
    return true;
  } else if (vdim == 6) {
    char* s = buffer;
    OnlineVertexSE3* v3 = dynamic_cast<OnlineVertexSE3*>(v);
    Vector3d eulerAngles =
        internal::toEuler(v3->updatedEstimate.matrix().topLeftCorner<3, 3>());
    const double& roll = eulerAngles(0);
    const double& pitch = eulerAngles(1);
    const double& yaw = eulerAngles(2);
    memcpy(s, "VERTEX_XYZRPY ", 14);
    s += 14;
    s += modp_itoa10(v->id(), s);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().x(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().y(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(v3->updatedEstimate.translation().z(), s, 6);
    *s++ = ' ';
    s += modp_dtoa(roll, s, 6);
    *s++ = ' ';
    s += modp_dtoa(pitch, s, 6);
    *s++ = ' ';
    s += modp_dtoa(yaw, s, 6);
    *s++ = '\n';
    cout.write(buffer, s - buffer);
    return true;
  }
  return false;
}

void G2oSlamInterface::setUpdateGraphEachN(int n) { _updateGraphEachN = n; }

G2oSlamInterface::SolveResult G2oSlamInterface::solve() {
  if (_nodesAdded >= _updateGraphEachN) {
    // decide on batch step or normal step
    _optimizer->batchStep = false;
    if ((int)_optimizer->vertices().size() - _lastBatchStep >= _batchEveryN) {
      _lastBatchStep = _optimizer->vertices().size();
      _optimizer->batchStep = true;
    }

    if (_firstOptimization) {
      if (!_optimizer->initializeOptimization()) {
        ngx::Log(ngx::Error, "initialization failed");
        return ERROR;
      }
    } else {
      if (!_optimizer->updateInitialization(_verticesAdded, _edgesAdded)) {
        ngx::Log(ngx::Error, "updating initialization failed");
        return ERROR;
      }
    }

    int currentIt = _optimizer->optimize(_incIterations, !_firstOptimization);
    (void)currentIt;
    _firstOptimization = false;
    _nodesAdded = 0;
    _verticesAdded.clear();
    _edgesAdded.clear();
    if (_optimizer->batchStep) return SOLVED_BATCH;
    return SOLVED;
  }

  return NOOP;
}

void G2oSlamInterface::setBatchSolveEachN(int n) { _batchEveryN = n; }

G2O_END_NAMESPACE
