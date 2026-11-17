// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H
#define G2O_GRAPH_OPTIMIZER_SPARSE_ONLINE_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/examples/interactive_slam/g2o_interactive_api.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

class Solver;

class G2O_INTERACTIVE_API SparseOptimizerOnline : public SparseOptimizer {
 public:
  explicit SparseOptimizerOnline(bool pcg = false);
  virtual ~SparseOptimizerOnline();

  int optimize(int iterations, bool online = false);

  virtual bool updateInitialization(HyperGraph::VertexSet& vset,
                                    HyperGraph::EdgeSet& eset);

  void update(double* update);

  virtual bool initSolver(int dimension, int batchEveryN);

 public:
  int slamDimension;

  HyperGraph::EdgeSet* newEdges;

  bool batchStep;
  bool vizWithGnuplot;

  virtual void gnuplotVisualization();

 protected:
  FILE* _gnuplot;
  bool _usePcg;
  Solver* _underlyingSolver;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
