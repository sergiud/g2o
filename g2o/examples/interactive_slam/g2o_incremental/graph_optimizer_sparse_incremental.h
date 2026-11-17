// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: GPL-3.0-or-later

#ifndef G2O_GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H
#define G2O_GRAPH_OPTIMIZER_SPARSE_INCREMENTAL_H

#include <g2o/core/sparse_block_matrix.h>
#include <g2o/examples/interactive_slam/g2o_incremental/linear_solver_cholmod_online.h>
#include <g2o/examples/interactive_slam/g2o_incremental_api.h>
#include <g2o/examples/interactive_slam/g2o_interactive/graph_optimizer_sparse_online.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  struct CholmodExt;

  class G2O_INCREMENTAL_API SparseOptimizerIncremental : public SparseOptimizerOnline
  {
    public:
      SparseOptimizerIncremental();
      ~SparseOptimizerIncremental();

      int optimize(int iterations, bool online = false);

      virtual bool updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset);

      virtual bool initSolver(int dimension, int batchEveryN);

    protected:
      SparseBlockMatrix<Eigen::MatrixXd> _updateMat;
      cholmod_common _cholmodCommon{};
      CholmodExt* _cholmodSparse;
      cholmod_factor* _cholmodFactor;
      cholmod_triplet* _permutedUpdate;
      cholmod_factor* L_;
      LinearSolverCholmodOnlineInterface* _solverInterface;

      HyperGraph::VertexSet _touchedVertices;
      Eigen::VectorXi _perm;
      Eigen::VectorXi _cmember;

      Eigen::VectorXi _tripletWorkspace;
      CholmodExt* _permutedUpdateAsSparse;

      bool computeCholeskyUpdate();
      void convertTripletUpdateToSparse();
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
