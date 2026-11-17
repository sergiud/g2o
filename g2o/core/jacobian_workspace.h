// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef JACOBIAN_WORKSPACE_H
#define JACOBIAN_WORKSPACE_H

#include <g2o/config.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/prefix.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cassert>
#include <vector>

G2O_START_NAMESPACE

struct OptimizableGraph;

/**
 * \brief provide memory workspace for computing the Jacobians
 *
 * The workspace is used by an OptimizableGraph to provide temporary memory
 * for computing the Jacobian of the error functions.
 * Before calling linearizeOplus on an edge, the workspace needs to be allocated
 * by calling allocate().
 */
class G2O_CORE_API JacobianWorkspace {
 public:
  typedef std::vector<VectorXD, Eigen::aligned_allocator<VectorXD> >
      WorkspaceVector;

 public:
  JacobianWorkspace();
  ~JacobianWorkspace();

  /**
   * allocate the workspace
   */
  bool allocate();

  /**
   * update the maximum required workspace needed by taking into account this
   * edge
   */
  void updateSize(const HyperGraph::Edge* e);

  /**
   * update the required workspace by looking at a full graph
   */
  void updateSize(const OptimizableGraph& graph);

  /**
   * manually update with the given parameters
   */
  void updateSize(int numVertices, int dimension);

  /**
   * return the workspace for a vertex in an edge
   */
  double* workspaceForVertex(int vertexIndex) {
    assert(vertexIndex >= 0 && (size_t)vertexIndex < _workspace.size() &&
           "Index out of bounds");
    return _workspace[vertexIndex].data();
  }

 protected:
  WorkspaceVector
      _workspace;  ///< the memory pre-allocated for computing the Jacobians
  int _maxNumVertices;  ///< the maximum number of vertices connected by a
                        ///< hyper-edge
  int _maxDimension;    ///< the maximum dimension (number of elements) for a
                        ///< Jacobian
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
