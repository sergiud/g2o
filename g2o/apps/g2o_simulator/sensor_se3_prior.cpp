// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_se3_prior.h"

#include <memory>
#include <utility>

G2O_START_NAMESPACE
using namespace std;

// SensorSE3Prior
SensorSE3Prior::SensorSE3Prior(const std::string& name_)
    : UnarySensor<Robot3D, EdgeSE3Prior>(name_) {
  _offsetParam = nullptr;
  _information.setIdentity();
  _information *= 1000;
  _information(2, 2) = 10;
  setInformation(_information);
}

void SensorSE3Prior::addParameters() {
  if (_offsetParam == nullptr) {
    auto offsetParam = std::make_unique<ParameterSE3Offset>();
    _offsetParam = offsetParam.get();
    assert(world());
    world()->addParameter(std::move(offsetParam));
  }
}

void SensorSE3Prior::addNoise(EdgeType* e) {
  EdgeType::ErrorVector _n = _sampler.generateSample();
  EdgeType::Measurement n = internal::fromVectorMQT(_n);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorSE3Prior::sense() {
  if (_offsetParam == nullptr) {
    return;
  }
  _robotPoseObject = nullptr;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (_robotPoseObject == nullptr) _robotPoseObject = *it;
    it++;
    count++;
  }
  if (_robotPoseObject == nullptr) return;
  _sensorPose = _robotPoseObject->vertex()->estimate() * _offsetParam->offset();
  std::unique_ptr<EdgeType> e = mkEdge();
  e->setParameterId(0, _offsetParam->id());
  if ((e != nullptr) && (graph() != nullptr)) {
    EdgeType* edge = e.get();
    // The graph resolves the parameters that the measurement needs.
    if (graph()->addEdge(std::move(e))) {
      edge->setMeasurementFromState();
      addNoise(edge);
    }
  }
}

G2O_END_NAMESPACE
