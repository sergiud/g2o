// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sclam_helpers.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/stuff/logger.h>
#include <g2o/types/data/data_queue.h>
#include <g2o/types/data/robot_laser.h>
#include <g2o/types/sclam2d/edge_se2_odom_differential_calib.h>
#include <g2o/types/sclam2d/odometry_measurement.h>
#include <g2o/types/sclam2d/vertex_odom_differential_params.h>

#include <memory>
#include <utility>

#include "gm2dl_io.h"
using namespace std;
using namespace Eigen;

G2O_START_NAMESPACE

static const double INFORMATION_SCALING_ODOMETRY = 100;

void addOdometryCalibLinksDifferential(SparseOptimizer& optimizer,
                                       const DataQueue& odomData) {
  auto odomParams = std::make_unique<VertexOdomDifferentialParams>();
  VertexOdomDifferentialParams* odomParamsVertex = odomParams.get();
  odomParamsVertex->setToOrigin();
  odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
  optimizer.addVertex(std::move(odomParams));

  // The new edges join the graph after the loop over its edges.
  std::vector<std::unique_ptr<EdgeSE2OdomDifferentialCalib>> odomCalibEdges;
  for (const auto& it : optimizer.edges()) {
    EdgeSE2SensorCalib* scanmatchEdge =
        dynamic_cast<EdgeSE2SensorCalib*>(it.get());
    if (scanmatchEdge == nullptr) continue;

    VertexSE2* r1 = dynamic_cast<VertexSE2*>(scanmatchEdge->vertices()[0]);
    VertexSE2* r2 = dynamic_cast<VertexSE2*>(scanmatchEdge->vertices()[1]);
    if (r2->id() - r1->id() != 1) {  // ignore non-incremental edges
      continue;
    }

    RobotLaser* rl1 = dynamic_cast<RobotLaser*>(r1->userData());
    RobotLaser* rl2 = dynamic_cast<RobotLaser*>(r2->userData());
    RobotLaser* odom1 =
        dynamic_cast<RobotLaser*>(odomData.findClosestData(rl1->timestamp()));
    RobotLaser* odom2 =
        dynamic_cast<RobotLaser*>(odomData.findClosestData(rl2->timestamp()));

    if (fabs(rl1->timestamp() - rl2->timestamp()) < 1e-7) {
      ngx::Log("strange egde {} <-> {}", r1->id(), r2->id());
      ngx::Log("rl1->timestamp() = {:f}\t rl2->timestamp() = {:f}",
               rl1->timestamp(), rl2->timestamp());
      ngx::Log("odom1->timestamp() = {:f}\t odom2->timestamp() = {:f}",
               odom1->timestamp(), odom2->timestamp());
    }

    // cerr << PVAR(odom1->odomPose().toVector().transpose()) << endl;

    SE2 odomMotion = odom1->odomPose().inverse() * odom2->odomPose();
    // cerr << PVAR(odomMotion.toVector().transpose()) << endl;
    // cerr << PVAR(scanmatchEdge->measurement().toVector().transpose()) <<
    // endl;

    auto e = std::make_unique<EdgeSE2OdomDifferentialCalib>();
    e->vertices()[0] = r1;
    e->vertices()[1] = r2;
    e->vertices()[2] = odomParamsVertex;

    MotionMeasurement mm(
        odomMotion.translation().x(), odomMotion.translation().y(),
        odomMotion.rotation().angle(), odom2->timestamp() - odom1->timestamp());
    e->setMeasurement(OdomConvert::convertToVelocity(mm));
    // cerr << PVAR(e->measurement()) << endl;

    e->information() = Matrix3d::Identity() * INFORMATION_SCALING_ODOMETRY;
    odomCalibEdges.push_back(std::move(e));
  }

  for (auto& odomCalibEdge : odomCalibEdges) {
    optimizer.addEdge(std::move(odomCalibEdge));
  }
}

void allocateSolverForSclam(SparseOptimizer& optimizer, bool levenberg) {
  typedef BlockSolver<BlockSolverTraits<-1, -1>> SclamBlockSolver;
  typedef LinearSolverCSparse<SclamBlockSolver::PoseMatrixType>
      SclamLinearSolver;

  auto linearSolver = std::make_unique<SclamLinearSolver>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver =
      std::make_unique<SclamBlockSolver>(std::move(linearSolver));
  std::unique_ptr<OptimizationAlgorithm> solver;
  if (levenberg) {
    solver = std::make_unique<OptimizationAlgorithmLevenberg>(
        std::move(blockSolver));
  } else {
    solver = std::make_unique<OptimizationAlgorithmGaussNewton>(
        std::move(blockSolver));
  }
  optimizer.setAlgorithm(std::move(solver));
}

G2O_END_NAMESPACE
