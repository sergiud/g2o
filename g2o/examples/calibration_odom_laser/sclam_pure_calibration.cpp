// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/examples/calibration_odom_laser/closed_form_calibration.h>
#include <g2o/examples/calibration_odom_laser/motion_information.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/stuff/timeutil.h>
#include <g2o/types/data/data_queue.h>
#include <g2o/types/data/robot_laser.h>
#include <g2o/types/sclam2d/odometry_measurement.h>
#include <g2o/types/sclam2d/vertex_odom_differential_params.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <csignal>
#include <fstream>
#include <iostream>
#include <map>

#include "edge_se2_pure_calib.h"
#include "gm2dl_io.h"
#include "sclam_helpers.h"

using namespace std;
using namespace g2o;

static Eigen::Vector2d linearSolution;

class VertexBaseline : public BaseVertex<1, double> {
 public:
  VertexBaseline() {}

  virtual void setToOriginImpl() { _estimate = 1.; }
  virtual void oplusImpl(const double* update) { _estimate += update[0]; }
  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }
};

class EdgeCalib
    : public BaseBinaryEdge<3, OdomAndLaserMotion, VertexSE2, VertexBaseline> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  EdgeCalib() {}

  void computeError() {
    const VertexSE2* laserOffset = dynamic_cast<const VertexSE2*>(_vertices[0]);
    const VertexBaseline* odomParams =
        dynamic_cast<const VertexBaseline*>(_vertices[1]);

    // get the calibrated motion given by the odometry
    double rl = -odomParams->estimate() * linearSolution(0);
    double rr = odomParams->estimate() * linearSolution(1);
    VelocityMeasurement calibratedVelocityMeasurment(
        measurement().velocityMeasurement.vl() * rl,
        measurement().velocityMeasurement.vr() * rr,
        measurement().velocityMeasurement.dt());
    MotionMeasurement mm = OdomConvert::convertToMotion(
        calibratedVelocityMeasurment, odomParams->estimate());
    SE2 Ku_ij;
    Ku_ij.fromVector(mm.measurement());

    SE2 laserMotionInRobotFrame = laserOffset->estimate() *
                                  measurement().laserMotion *
                                  laserOffset->estimate().inverse();
    SE2 delta = Ku_ij.inverse() * laserMotionInRobotFrame;
    _error = delta.toVector();
  }

  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }
};

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  bool fixLaser;
  int maxIterations;
  bool verbose;
  string inputFilename;
  string outputfilename;
  string rawFilename;
  string odomTestFilename;
  string dumpGraphFilename;
  // command line parsing
  CommandArgs commandLineArguments;
  commandLineArguments.param("i", maxIterations, 10, "perform n iterations");
  commandLineArguments.param("v", verbose, false,
                             "verbose output of the optimization process");
  commandLineArguments.param("o", outputfilename, "",
                             "output final version of the graph");
  commandLineArguments.param("test", odomTestFilename, "",
                             "apply odometry calibration to some test data");
  commandLineArguments.param("dump", dumpGraphFilename, "",
                             "write the graph to the disk");
  commandLineArguments.param("fixLaser", fixLaser, false,
                             "keep the laser offset fixed during optimization");
  commandLineArguments.paramLeftOver("gm2dl-input", inputFilename, "",
                                     "gm2dl file which will be processed");
  commandLineArguments.paramLeftOver("raw-log", rawFilename, "",
                                     "raw log file containing the odometry");

  commandLineArguments.parseArgs(argc, argv);

  SparseOptimizer optimizer;
  optimizer.setVerbose(verbose);

  allocateSolverForSclam(optimizer);

  // loading
  DataQueue odometryQueue;
  int numLaserOdom = Gm2dlIO::readRobotLaser(rawFilename, odometryQueue);
  if (numLaserOdom == 0) {
    ngx::Log(ngx::Warning, "No raw information read");
    return 0;
  }
  ngx::Log("Read {} laser readings from file", numLaserOdom);

  Eigen::Vector3d odomCalib(1., 1., 1.);
  SE2 initialLaserPose;
  DataQueue robotLaserQueue;
  int numRobotLaser = Gm2dlIO::readRobotLaser(inputFilename, robotLaserQueue);
  if (numRobotLaser == 0) {
    ngx::Log(ngx::Warning, "No robot laser read");
    return 0;
  } else {
    RobotLaser* rl =
        dynamic_cast<RobotLaser*>(robotLaserQueue.buffer().begin()->second);
    initialLaserPose = rl->odomPose().inverse() * rl->laserPose();
    ngx::Log("initialLaserPose.toVector().transpose() = {}",
             Streamed{initialLaserPose.toVector().transpose()});
  }

  // adding the measurements
  vector<MotionInformation, Eigen::aligned_allocator<MotionInformation> >
      motions;
  {
    auto it = robotLaserQueue.buffer().begin();
    auto prevIt = it++;
    for (; it != robotLaserQueue.buffer().end(); ++it) {
      MotionInformation mi;
      RobotLaser* prevLaser = dynamic_cast<RobotLaser*>(prevIt->second);
      RobotLaser* curLaser = dynamic_cast<RobotLaser*>(it->second);
      mi.laserMotion = prevLaser->laserPose().inverse() * curLaser->laserPose();
      // get the motion of the robot in that time interval
      RobotLaser* prevOdom = dynamic_cast<RobotLaser*>(
          odometryQueue.findClosestData(prevLaser->timestamp()));
      RobotLaser* curOdom = dynamic_cast<RobotLaser*>(
          odometryQueue.findClosestData(curLaser->timestamp()));
      mi.odomMotion = prevOdom->odomPose().inverse() * curOdom->odomPose();
      mi.timeInterval = prevOdom->timestamp() - curOdom->timestamp();
      prevIt = it;
      motions.push_back(mi);
    }
  }

  if (1) {
    auto* laserOffset = new VertexSE2;
    laserOffset->setId(Gm2dlIO::ID_LASERPOSE);
    laserOffset->setEstimate(initialLaserPose);
    optimizer.addVertex(laserOffset);
    auto* odomParamsVertex = new VertexOdomDifferentialParams;
    odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
    odomParamsVertex->setEstimate(Eigen::Vector3d(1., 1., 1.));
    optimizer.addVertex(odomParamsVertex);
    for (auto& motion : motions) {
      const SE2& odomMotion = motion.odomMotion;
      const SE2& laserMotion = motion.laserMotion;
      const double& timeInterval = motion.timeInterval;
      // add the edge
      MotionMeasurement mm(odomMotion.translation().x(),
                           odomMotion.translation().y(),
                           odomMotion.rotation().angle(), timeInterval);
      OdomAndLaserMotion meas;
      meas.velocityMeasurement = OdomConvert::convertToVelocity(mm);
      meas.laserMotion = laserMotion;
      auto* calibEdge = new EdgeSE2PureCalib;
      calibEdge->setVertex(0, laserOffset);
      calibEdge->setVertex(1, odomParamsVertex);
      calibEdge->setInformation(Eigen::Matrix3d::Identity());
      calibEdge->setMeasurement(meas);
      if (!optimizer.addEdge(calibEdge)) {
        ngx::Log(ngx::Error, "Error adding calib edge");
        delete calibEdge;
      }
    }

    if (fixLaser) {
      ngx::Log("Fix position of the laser offset");
      laserOffset->setFixed(true);
    }

    ngx::Log("\nPerforming full non-linear estimation");
    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    optimizer.optimize(maxIterations);
    ngx::Log("Calibrated laser offset (x, y, theta):{}",
             laserOffset->estimate().toVector().transpose());
    odomCalib = odomParamsVertex->estimate();
    ngx::Log("Odometry parameters (scaling factors (v_l, v_r, b)): {}",
             odomParamsVertex->estimate().transpose());
    optimizer.clear();
  }

  // linear least squares for some parameters
  {
    Eigen::MatrixXd A(motions.size(), 2);
    Eigen::VectorXd x(motions.size());
    for (size_t i = 0; i < motions.size(); ++i) {
      const SE2& odomMotion = motions[i].odomMotion;
      const SE2& laserMotion = motions[i].laserMotion;
      const double& timeInterval = motions[i].timeInterval;
      MotionMeasurement mm(odomMotion.translation().x(),
                           odomMotion.translation().y(),
                           odomMotion.rotation().angle(), timeInterval);
      VelocityMeasurement velMeas = OdomConvert::convertToVelocity(mm);
      A(i, 0) = velMeas.vl() * timeInterval;
      A(i, 1) = velMeas.vr() * timeInterval;
      x(i) = laserMotion.rotation().angle();
    }
    // linearSolution = (A.transpose() * A).inverse() * A.transpose() * x;
    linearSolution = A.colPivHouseholderQr().solve(x);
    // cout << PVAR(linearSolution.transpose()) << endl;
  }

  // constructing non-linear least squares
  auto* laserOffset = new VertexSE2;
  laserOffset->setId(Gm2dlIO::ID_LASERPOSE);
  laserOffset->setEstimate(initialLaserPose);
  optimizer.addVertex(laserOffset);
  auto* odomParamsVertex = new VertexBaseline;
  odomParamsVertex->setId(Gm2dlIO::ID_ODOMCALIB);
  odomParamsVertex->setEstimate(1.);
  optimizer.addVertex(odomParamsVertex);
  for (auto& motion : motions) {
    const SE2& odomMotion = motion.odomMotion;
    const SE2& laserMotion = motion.laserMotion;
    const double& timeInterval = motion.timeInterval;
    // add the edge
    MotionMeasurement mm(odomMotion.translation().x(),
                         odomMotion.translation().y(),
                         odomMotion.rotation().angle(), timeInterval);
    OdomAndLaserMotion meas;
    meas.velocityMeasurement = OdomConvert::convertToVelocity(mm);
    meas.laserMotion = laserMotion;
    auto* calibEdge = new EdgeCalib;
    calibEdge->setVertex(0, laserOffset);
    calibEdge->setVertex(1, odomParamsVertex);
    calibEdge->setInformation(Eigen::Matrix3d::Identity());
    calibEdge->setMeasurement(meas);
    if (!optimizer.addEdge(calibEdge)) {
      ngx::Log(ngx::Error, "Error adding calib edge");
      delete calibEdge;
    }
  }

  if (fixLaser) {
    ngx::Log("Fix position of the laser offset");
    laserOffset->setFixed(true);
  }

  ngx::Log("\nPerforming partial non-linear estimation");
  optimizer.initializeOptimization();
  optimizer.computeActiveErrors();
  optimizer.optimize(maxIterations);
  ngx::Log("Calibrated laser offset (x, y, theta):{}",
           laserOffset->estimate().toVector().transpose());
  odomCalib(0) = -1. * linearSolution(0) * odomParamsVertex->estimate();
  odomCalib(1) = linearSolution(1) * odomParamsVertex->estimate();
  odomCalib(2) = odomParamsVertex->estimate();
  ngx::Log("Odometry parameters (scaling factors (v_l, v_r, b)): {}",
           odomCalib.transpose());

  {
    SE2 closedFormLaser;
    Eigen::Vector3d closedFormOdom;
    ClosedFormCalibration::calibrate(motions, closedFormLaser, closedFormOdom);
    ngx::Log("\nObtaining closed form solution");
    ngx::Log("Calibrated laser offset (x, y, theta):{}",
             closedFormLaser.toVector().transpose());
    ngx::Log("Odometry parameters (scaling factors (v_l, v_r, b)): {}",
             closedFormOdom.transpose());
  }

  if (!dumpGraphFilename.empty()) {
    ngx::Log("Writing {}", dumpGraphFilename);
    optimizer.save(dumpGraphFilename.c_str());
    ngx::Log("done.");
  }

  // optional input of a separate file for applying the odometry calibration
  if (!odomTestFilename.empty()) {
    DataQueue testRobotLaserQueue;
    int numTestOdom =
        Gm2dlIO::readRobotLaser(odomTestFilename, testRobotLaserQueue);
    if (numTestOdom == 0) {
      ngx::Log(ngx::Error, "Unable to read test data");
    } else {
      ofstream rawStream("odometry_raw.txt");
      ofstream calibratedStream("odometry_calibrated.txt");
      RobotLaser* prev = dynamic_cast<RobotLaser*>(
          testRobotLaserQueue.buffer().begin()->second);
      SE2 prevCalibratedPose = prev->odomPose();

      for (const auto& it : testRobotLaserQueue.buffer()) {
        RobotLaser* cur = dynamic_cast<RobotLaser*>(it.second);
        assert(cur);

        double dt = cur->timestamp() - prev->timestamp();
        SE2 motion = prev->odomPose().inverse() * cur->odomPose();

        // convert to velocity measurement
        MotionMeasurement motionMeasurement(motion.translation().x(),
                                            motion.translation().y(),
                                            motion.rotation().angle(), dt);
        VelocityMeasurement velocityMeasurement =
            OdomConvert::convertToVelocity(motionMeasurement);

        // apply calibration
        VelocityMeasurement calibratedVelocityMeasurment = velocityMeasurement;
        calibratedVelocityMeasurment.setVl(odomCalib(0) *
                                           calibratedVelocityMeasurment.vl());
        calibratedVelocityMeasurment.setVr(odomCalib(1) *
                                           calibratedVelocityMeasurment.vr());
        MotionMeasurement mm = OdomConvert::convertToMotion(
            calibratedVelocityMeasurment, odomCalib(2));

        // combine calibrated odometry with the previous pose
        SE2 remappedOdom;
        remappedOdom.fromVector(mm.measurement());
        SE2 calOdomPose = prevCalibratedPose * remappedOdom;

        // write output
        rawStream << prev->odomPose().translation().x() << " "
                  << prev->odomPose().translation().y() << " "
                  << prev->odomPose().rotation().angle() << endl;
        calibratedStream << calOdomPose.translation().x() << " "
                         << calOdomPose.translation().y() << " "
                         << calOdomPose.rotation().angle() << endl;

        prevCalibratedPose = calOdomPose;
        prev = cur;
      }
    }
  }

  return 0;
}
