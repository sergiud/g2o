// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_pointxy_offset.h"

#include <g2o/core/factory.h>

#include <memory>
#include <utility>

G2O_START_NAMESPACE
using namespace std;
using namespace Eigen;

// SensorPointXYOffset
SensorPointXYOffset::SensorPointXYOffset(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2PointXYOffset, WorldObjectPointXY>(name_) {
  _offsetParam = nullptr;
  _information.setIdentity();
  _information *= 1000.;
  setInformation(_information);
}

bool SensorPointXYOffset::isVisible(SensorPointXYOffset::WorldObjectType* to) {
  if (_robotPoseObject == nullptr) return false;
  assert(to && to->vertex());
  VertexType* v = to->vertex();
  VertexType::EstimateType pose = v->estimate();
  VertexType::EstimateType delta = _sensorPose.inverse() * pose;
  Vector2d translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > _maxRange2) return false;
  if (range2 < _minRange2) return false;
  translation.normalize();
  // the cameras have the z in front
  double bearing = atan2(translation.y(), translation.x());
  if (fabs(bearing) > _fov) return false;
  // cerr <<"a";
  return true;
}

void SensorPointXYOffset::addParameters() {
  if (_offsetParam == nullptr) {
    auto offsetParam = std::make_unique<ParameterSE2Offset>();
    _offsetParam = offsetParam.get();
    assert(world());
    world()->addParameter(std::move(offsetParam));
  }
}

void SensorPointXYOffset::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

void SensorPointXYOffset::sense() {
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
  for (const auto& it : world()->objects()) {
    WorldObjectType* o = dynamic_cast<WorldObjectType*>(it.get());
    if ((o != nullptr) && isVisible(o)) {
      std::unique_ptr<EdgeType> e = mkEdge(o);
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
  }
}

G2O_END_NAMESPACE
