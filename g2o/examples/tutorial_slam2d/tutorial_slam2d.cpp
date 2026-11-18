// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/stuff/logger.h>

#include <cmath>
#include <memory>
#include <utility>

#include "edge_se2.h"
#include "edge_se2_pointxy.h"
#include "simulator.h"
#include "vertex_point_xy.h"
#include "vertex_se2.h"

using namespace std;
using namespace g2o;
using namespace g2o::tutorial;

int main() {
  g2o::initializeLogging("tutorial_slam2d");
  // TODO simulate different sensor offset
  // simulate a robot observing landmarks while travelling on a grid
  SE2 sensorOffsetTransf(0.2, 0.1, -0.1);
  int numNodes = 300;
  Simulator simulator;
  simulator.simulate(numNodes, sensorOffsetTransf);

  /*********************************************************************************
   * creating the optimization problem
   ********************************************************************************/

  typedef BlockSolver<BlockSolverTraits<-1, -1> > SlamBlockSolver;
  typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;

  // allocating the optimizer
  SparseOptimizer optimizer;
  auto linearSolver = std::make_unique<SlamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<SlamBlockSolver>(std::move(linearSolver));
  auto solver = std::make_unique<OptimizationAlgorithmGaussNewton>(
      std::move(blockSolver));

  optimizer.setAlgorithm(std::move(solver));

  // add the parameter representing the sensor offset
  auto sensorOffsetOwner = std::make_unique<ParameterSE2Offset>();
  auto* sensorOffset = sensorOffsetOwner.get();
  sensorOffset->setOffset(sensorOffsetTransf);
  sensorOffset->setId(0);
  optimizer.addParameter(std::move(sensorOffsetOwner));

  // adding the odometry to the optimizer
  // first adding all the vertices
  ngx::Log("Optimization: Adding robot poses");
  for (const auto& p : simulator.poses()) {
    const SE2& t = p.simulatorPose;
    auto robot = std::make_unique<VertexSE2>();
    robot->setId(p.id);
    robot->setEstimate(t);
    optimizer.addVertex(std::move(robot));
  }
  ngx::Log("done.");

  // second add the odometry constraints
  ngx::Log("Optimization: Adding odometry measurements");
  for (const auto& simEdge : simulator.odometry()) {
    auto odometry = std::make_unique<EdgeSE2>();
    odometry->vertices()[0] = optimizer.vertex(simEdge.from);
    odometry->vertices()[1] = optimizer.vertex(simEdge.to);
    odometry->setMeasurement(simEdge.simulatorTransf);
    odometry->setInformation(simEdge.information);
    optimizer.addEdge(std::move(odometry));
  }
  ngx::Log("done.");

  // add the landmark observations
  ngx::Log("Optimization: add landmark vertices");
  for (const auto& l : simulator.landmarks()) {
    auto landmark = std::make_unique<VertexPointXY>();
    landmark->setId(l.id);
    landmark->setEstimate(l.simulatedPose);
    optimizer.addVertex(std::move(landmark));
  }
  ngx::Log("done.");

  ngx::Log("Optimization: add landmark observations");
  for (const auto& simEdge : simulator.landmarkObservations()) {
    auto landmarkObservation = std::make_unique<EdgeSE2PointXY>();
    landmarkObservation->vertices()[0] = optimizer.vertex(simEdge.from);
    landmarkObservation->vertices()[1] = optimizer.vertex(simEdge.to);
    landmarkObservation->setMeasurement(simEdge.simulatorMeas);
    landmarkObservation->setInformation(simEdge.information);
    landmarkObservation->setParameterId(0, sensorOffset->id());
    optimizer.addEdge(std::move(landmarkObservation));
  }
  ngx::Log("done.");

  /*********************************************************************************
   * optimization
   ********************************************************************************/

  // dump initial state to the disk
  optimizer.save("tutorial_before.g2o");

  // prepare and run the optimization
  // fix the first robot pose to account for gauge freedom
  VertexSE2* firstRobotPose = dynamic_cast<VertexSE2*>(optimizer.vertex(0));
  firstRobotPose->setFixed(true);
  optimizer.setVerbose(true);

  ngx::Log("Optimizing");
  optimizer.initializeOptimization();
  optimizer.optimize(10);
  ngx::Log("done.");

  optimizer.save("tutorial_after.g2o");

  // freeing the graph memory
  optimizer.clear();

  return 0;
}
