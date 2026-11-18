// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/optimizable_graph.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>

#include <Eigen/Core>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <utility>

#include "simulator3d.h"

// #define _POSE_SENSOR_OFFSET
// #define _POSE_PRIOR_SENSOR

using namespace g2o;
using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  CommandArgs arg;
  int nlandmarks;
  int simSteps;
  double worldSize;
  bool hasOdom;
  bool hasPoseSensor;
  bool hasPointSensor;
  bool hasPointDepthSensor;
  bool hasPointDisparitySensor;
  bool hasCompass;
  bool hasGPS;

  std::string outputFilename;
  arg.param("simSteps", simSteps, 100, "number of simulation steps");
  arg.param("nLandmarks", nlandmarks, 1000, "number of landmarks");
  arg.param("worldSize", worldSize, 25.0, "size of the world");
  arg.param("hasOdom", hasOdom, false, "the robot has an odometry");
  arg.param("hasPointSensor", hasPointSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDepthSensor", hasPointDepthSensor, false,
            "the robot has a point sensor");
  arg.param("hasPointDisparitySensor", hasPointDisparitySensor, false,
            "the robot has a point sensor");
  arg.param("hasPoseSensor", hasPoseSensor, false,
            "the robot has a pose sensor");
  arg.param("hasCompass", hasCompass, false, "the robot has a compass");
  arg.param("hasGPS", hasGPS, false, "the robot has a GPS");
  arg.paramLeftOver("graph-output", outputFilename, "simulator_out.g2o",
                    "graph file which will be written", true);

  arg.parseArgs(argc, argv);

  std::mt19937 generator;
  OptimizableGraph graph;
  World world(&graph);
  for (int i = 0; i < nlandmarks; i++) {
    auto landmark = std::make_unique<WorldObjectTrackXYZ>();
    double x = sampleUniform(-.5, .5, &generator) * worldSize;
    double y = sampleUniform(-.5, .5, &generator) * worldSize;
    double z = sampleUniform(-.5, .5);
    landmark->vertex()->setEstimate(Vector3d(x, y, z));
    world.addWorldObject(std::move(landmark));
  }
  Robot3D robot(&world, "myRobot");
  world.addRobot(&robot);

  stringstream ss;
  ss << "-ws" << worldSize;
  ss << "-nl" << nlandmarks;
  ss << "-steps" << simSteps;

  if (hasOdom) {
    auto odometrySensor = std::make_unique<SensorOdometry3D>("odometry");
    robot.addSensor(std::move(odometrySensor));
    ss << "-odom";
  }

  if (hasPointSensor) {
    auto pointSensorOwner = std::make_unique<SensorPointXYZ>("pointSensor");
    SensorPointXYZ* pointSensor = pointSensorOwner.get();
    pointSensor->setFov(M_PI / 4);
    robot.addSensor(std::move(pointSensorOwner));
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    pointSensor->setMaxRange(2.);
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    pointSensor->offsetParam()->setOffset(cameraPose);
    ss << "-pointXYZ";
  }

  if (hasPointDisparitySensor) {
    auto disparitySensorOwner =
        std::make_unique<SensorPointXYZDisparity>("disparitySensor");
    SensorPointXYZDisparity* disparitySensor = disparitySensorOwner.get();
    disparitySensor->setFov(M_PI / 4);
    disparitySensor->setMinRange(0.5);
    disparitySensor->setMaxRange(2.);
    robot.addSensor(std::move(disparitySensorOwner));
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    disparitySensor->offsetParam()->setOffset(cameraPose);
    ss << "-disparity";
  }

  if (hasPointDepthSensor) {
    auto depthSensorOwner =
        std::make_unique<SensorPointXYZDepth>("depthSensor");
    SensorPointXYZDepth* depthSensor = depthSensorOwner.get();
    depthSensor->setFov(M_PI / 4);
    depthSensor->setMinRange(0.5);
    depthSensor->setMaxRange(2.);
    robot.addSensor(std::move(depthSensorOwner));
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    depthSensor->offsetParam()->setOffset(cameraPose);
    ss << "-depth";
  }

  if (hasPoseSensor) {
    auto poseSensorOwner = std::make_unique<SensorPose3D>("poseSensor");
    SensorPose3D* poseSensor = poseSensorOwner.get();
    robot.addSensor(std::move(poseSensorOwner));
    poseSensor->setMaxRange(5);
    ss << "-pose";
  }

#ifdef _POSE_PRIOR_SENSOR
  auto posePriorSensorOwner =
      std::make_unique<SensorSE3Prior>("posePriorSensor");
  SensorSE3Prior& posePriorSensor = *posePriorSensorOwner;
  robot.addSensor(std::move(posePriorSensorOwner));
  {
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    posePriorSensor.offsetParam()->setOffset(cameraPose);
  }
#endif

#ifdef _POSE_SENSOR_OFFSET
  auto poseSensorOwner = std::make_unique<SensorPose3DOffset>("poseSensor");
  SensorPose3DOffset& poseSensor = *poseSensorOwner;
  poseSensor.setFov(M_PI / 4);
  poseSensor.setMinRange(0.5);
  poseSensor.setMaxRange(5);
  robot.addSensor(std::move(poseSensorOwner));
  if (0) {
    Eigen::Isometry3d cameraPose;
    Eigen::Matrix3d R;
    R << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    cameraPose = R;
    cameraPose.translation() = Vector3d(0., 0., 0.3);
    poseSensor.offsetParam1()->setOffset(cameraPose);
    poseSensor.offsetParam2()->setOffset(cameraPose);
  }
#endif

  robot.move(Eigen::Isometry3d::Identity());
  double pStraight = 0.7;
  Eigen::Isometry3d moveStraight = Eigen::Isometry3d::Identity();
  moveStraight.translation() = Vector3d(1., 0., 0.);
  double pLeft = 0.15;
  Eigen::Isometry3d moveLeft = Eigen::Isometry3d::Identity();
  moveLeft = AngleAxisd(M_PI / 2, Vector3d::UnitZ());
  // double pRight=0.15;
  Eigen::Isometry3d moveRight = Eigen::Isometry3d::Identity();
  moveRight = AngleAxisd(-M_PI / 2, Vector3d::UnitZ());

  Eigen::Matrix3d dtheta = Eigen::Matrix3d::Identity();
  for (int i = 0; i < simSteps; i++) {
    bool boundariesReached = true;
    Vector3d dt;
    const Eigen::Isometry3d& pose = robot.pose();
    if (pose.translation().x() < -.5 * worldSize) {
      dtheta = AngleAxisd(0, Vector3d::UnitZ());
    } else if (pose.translation().x() > .5 * worldSize) {
      dtheta = AngleAxisd(-M_PI, Vector3d::UnitZ());
    } else if (pose.translation().y() < -.5 * worldSize) {
      dtheta = AngleAxisd(M_PI / 2, Vector3d::UnitZ());
    } else if (pose.translation().y() > .5 * worldSize) {
      dtheta = AngleAxisd(-M_PI / 2, Vector3d::UnitZ());
    } else {
      boundariesReached = false;
    }

    Eigen::Isometry3d move = Eigen::Isometry3d::Identity();
    if (boundariesReached) {
      Eigen::Matrix3d mTheta = pose.rotation().inverse() * dtheta;
      move = mTheta;
      AngleAxisd aa(mTheta);
      if (aa.angle() < std::numeric_limits<double>::epsilon()) {
        move.translation() = Vector3d(1., 0., 0.);
      }
    } else {
      double sampled = sampleUniform();
      if (sampled < pStraight)
        move = moveStraight;
      else if (sampled < pStraight + pLeft)
        move = moveLeft;
      else
        move = moveRight;
    }

    // select a random move of the robot
    robot.relativeMove(move);
    // do a sense
    robot.sense();
  }
  string fname = outputFilename + ss.str() + ".g2o";
  // ofstream testStream(fname.c_str());
  ofstream testStream(outputFilename.c_str());
  graph.save(testStream);
}
