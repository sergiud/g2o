// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_odometry2d.h"

#include <g2o/stuff/logger.h>

#include <cassert>
#include <memory>
#include <utility>

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
    ngx::Log(ngx::Error, "trajectory empty");
    return;
  }
  _robotPoseObject = pprev;
  std::unique_ptr<EdgeType> e = mkEdge(pcurr);
  if (e != nullptr) {
    e->setMeasurementFromState();
    addNoise(e.get());
    if (graph() != nullptr) graph()->addEdge(std::move(e));
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
