// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_odometry2d.h"

#include <cassert>
#include <iostream>

// Robot2D
G2O_START_NAMESPACE
using namespace std;

SensorOdometry2D::SensorOdometry2D(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_) {}

void SensorOdometry2D::sense() {
  if (robot() == nullptr) return;

  RobotType* r = dynamic_cast<RobotType*>(robot());
  if (r == nullptr) return;

  PoseObject *pprev = nullptr, *pcurr = nullptr;
  auto it = r->trajectory().rbegin();
  if (it != r->trajectory().rend()) {
    pcurr = *it;
    it++;
  }
  if (it != r->trajectory().rend()) {
    pprev = *it;
    it++;
  }
  if (!((pcurr != nullptr) && (pprev != nullptr))) {
    cerr << __PRETTY_FUNCTION__ << ": fatal, trajectory empty" << endl;
    return;
  }
  _robotPoseObject = pprev;
  EdgeType* e = mkEdge(pcurr);
  if (e != nullptr) {
    e->setMeasurementFromState();
    addNoise(e);
    if (graph() != nullptr) graph()->addEdge(e);
  }
  _robotPoseObject = pcurr;
}

void SensorOdometry2D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = _sampler.generateSample();
  EdgeType::Measurement n;
  n.fromVector(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

G2O_END_NAMESPACE
