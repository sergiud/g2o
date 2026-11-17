// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#include "gtest/gtest.h"

#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> > SlamBlockSolver;
typedef g2o::LinearSolverEigen<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

static g2o::SparseOptimizer* createOptimizer()
{
  // Initialize the SparseOptimizer
  g2o::SparseOptimizer* mOptimizer = new g2o::SparseOptimizer();
  SlamLinearSolver* linearSolver = new SlamLinearSolver();
  linearSolver->setBlockOrdering(false);
  SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
  mOptimizer->setAlgorithm(new g2o::OptimizationAlgorithmGaussNewton(blockSolver));
  return mOptimizer;
}

TEST(General, BinaryEdgeConstructor)
{
  g2o::EdgeSE3 e1;
  ASSERT_EQ(NULL, e1.vertices()[0]);
  ASSERT_EQ(NULL, e1.vertices()[1]);

  g2o::EdgeSE2 e2;
  ASSERT_EQ(NULL, e2.vertices()[0]);
  ASSERT_EQ(NULL, e2.vertices()[1]);
}

TEST(General, GraphAddVertex)
{
  g2o::SparseOptimizer* optimizer = createOptimizer();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_EQ(optimizer, v1->graph());
  ASSERT_EQ(size_t(1), optimizer->vertices().size());
  ASSERT_FALSE(optimizer->addVertex(v1));
  ASSERT_EQ(size_t(1), optimizer->vertices().size());

  {
    g2o::VertexSE2* v2 = new g2o::VertexSE2();
    v2->setId(0);
    ASSERT_FALSE(optimizer->addVertex(v2));
    ASSERT_EQ(size_t(1), optimizer->vertices().size());
    ASSERT_EQ(NULL, v2->graph());
    delete v2;
  }

  delete optimizer;
}

TEST(General, GraphAddEdge)
{
  g2o::SparseOptimizer* optimizer = createOptimizer();

  g2o::VertexSE2* v1 = new g2o::VertexSE2();
  v1->setId(0);
  g2o::VertexSE2* v2 = new g2o::VertexSE2();
  v2->setId(1);

  ASSERT_TRUE(optimizer->addVertex(v1));
  ASSERT_TRUE(optimizer->addVertex(v2));

  g2o::EdgeSE2* e1 = new g2o::EdgeSE2();
  e1->setVertex(0, v1);
  e1->setVertex(1, v2);
  ASSERT_TRUE(optimizer->addEdge(e1));
  ASSERT_EQ(optimizer, e1->graph());
  ASSERT_FALSE(optimizer->addEdge(e1));
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().size());
  ASSERT_EQ(size_t(1), v1->edges().count(e1));
  ASSERT_EQ(size_t(1), v2->edges().count(e1));

  g2o::EdgeSE2* e2 = new g2o::EdgeSE2();
  ASSERT_FALSE(optimizer->addEdge(e2)) << "Adding edge with unset vertices was possible";
  ASSERT_EQ(size_t(1), optimizer->edges().size());
  ASSERT_EQ(NULL, e2->graph());

  delete optimizer;
}
