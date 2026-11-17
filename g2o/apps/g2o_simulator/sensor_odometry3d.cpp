// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_odometry3d.h"

#include <g2o/stuff/logger.h>
#include <g2o/types/slam3d/isometry3d_mappings.h>

G2O_START_NAMESPACE
using namespace std;

SensorOdometry3D::SensorOdometry3D(const std::string& name_)
    : BinarySensor<Robot3D, EdgeSE3, WorldObjectSE3>(name_) {
  _information.setIdentity();
  _information *= 100;
  _information(3, 3) = 10000;
  _information(4, 4) = 10000;
  _information(5, 5) = 10000;
  setInformation(_information);
}

void SensorOdometry3D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = _sampler.generateSample();
  EdgeType::Measurement n = internal::fromVectorMQT(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorOdometry3D::sense() {
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
  EdgeType* e = mkEdge(pcurr);
  if (e != nullptr) {
    if (graph() != nullptr) {
      graph()->addEdge(e);
      e->setMeasurementFromState();
      addNoise(e);
    }
  }
  _robotPoseObject = pcurr;
}

G2O_END_NAMESPACE
