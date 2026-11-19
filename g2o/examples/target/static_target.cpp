// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

// This example consists of a single static target which sits in one
// place and does not move; in effect it has a "GPS" which measures
// its position

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/sampler.h>
#include <g2o/stuff/streamed.h>

#include <Eigen/StdVector>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include "targetTypes3D.hpp"

using namespace Eigen;
using namespace std;
using namespace g2o;

int main() {
  g2o::initializeLogging("static_target");
  // Set up the optimiser
  SparseOptimizer optimizer;
  optimizer.setVerbose(false);

  // Create the block solver - the dimensions are specified because
  // 3D observations marginalise to a 3D estimate
  typedef BlockSolver<BlockSolverTraits<3, 3>> BlockSolver_3_3;
  auto linearSolver =
      std::make_unique<LinearSolverCholmod<BlockSolver_3_3::PoseMatrixType>>();
  auto blockSolver = std::make_unique<BlockSolver_3_3>(std::move(linearSolver));
  auto solver = std::make_unique<OptimizationAlgorithmGaussNewton>(
      std::move(blockSolver));
  optimizer.setAlgorithm(std::move(solver));

  // Sample the actual location of the target
  Vector3d truePoint(sampleUniform(-500, 500), sampleUniform(-500, 500),
                     sampleUniform(-500, 500));

  // Construct vertex which corresponds to the actual point of the target
  auto positionOwner = std::make_unique<VertexPosition3D>();
  auto* position = positionOwner.get();
  position->setId(0);
  optimizer.addVertex(std::move(positionOwner));

  // Now generate some noise corrupted measurements; for simplicity
  // these are uniformly distributed about the true target. These are
  // modelled as a unary edge because they do not like to, say,
  // another node in the map.
  int numMeasurements = 10;
  double noiseLimit = sqrt(12.);
  double noiseSigma = noiseLimit * noiseLimit / 12.0;

  for (int i = 0; i < numMeasurements; i++) {
    Vector3d measurement =
        truePoint + Vector3d(sampleUniform(-0.5, 0.5) * noiseLimit,
                             sampleUniform(-0.5, 0.5) * noiseLimit,
                             sampleUniform(-0.5, 0.5) * noiseLimit);
    auto goe = std::make_unique<GPSObservationPosition3DEdge>();
    goe->setVertex(0, position);
    goe->setMeasurement(measurement);
    goe->setInformation(Matrix3d::Identity() / noiseSigma);
    optimizer.addEdge(std::move(goe));
  }

  // Configure and set things going
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  optimizer.optimize(5);

  cout << "truePoint=\n" << truePoint << endl;

  ngx::Log("computed estimate=\n{}",
           Streamed(dynamic_cast<VertexPosition3D*>(
                        optimizer.vertices().find(0)->second.get())
                        ->estimate()));

  // position->setMarginalized(true);

  SparseBlockMatrix<MatrixXd> spinv;

  optimizer.computeMarginals(spinv, position);

  // optimizer.solver()->computeMarginals();

  // covariance
  //
  cout << "covariance\n" << spinv << endl;

  const MatrixXd* covariance = spinv.block(0, 0);
  if (covariance == nullptr) {
    ngx::Log(ngx::Error, "no covariance computed for the target");
    return EXIT_FAILURE;
  }
  cout << *covariance << endl;
}
