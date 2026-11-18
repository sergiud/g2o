// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_line3d.h"

#include <memory>
#include <utility>

G2O_START_NAMESPACE
using namespace std;

// SensorLine3D
SensorLine3D::SensorLine3D(const std::string& name_)
    : BinarySensor<Robot3D, EdgeSE3Line3D, WorldObjectLine3D>(name_) {
  _offsetParam = 0;
  _information.setIdentity();
  _information *= 1e9;
  //_information(2,2)=10;
  setInformation(_information);
}

bool SensorLine3D::isVisible(SensorLine3D::WorldObjectType* to) {
  if (!_robotPoseObject) return false;
  assert(to && to->vertex());
  VertexType* v = to->vertex();
  VertexType::EstimateType pose = v->estimate();
  VertexType::EstimateType delta = _sensorPose.inverse() * pose;
  Vector3d translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > _maxRange2) return false;
  if (range2 < _minRange2) return false;
  translation.normalize();
  // the cameras have the z in front
  double bearing = acos(translation.z());
  if (fabs(bearing) > _fov) return false;
  return true;
}

void SensorLine3D::addParameters() {
  if (_offsetParam == nullptr) {
    auto offsetParam = std::make_unique<ParameterSE3Offset>();
    _offsetParam = offsetParam.get();
    assert(world());
    world()->addParameter(std::move(offsetParam));
  }
}

void SensorLine3D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

void SensorLine3D::sense() {
  if (!_offsetParam) {
    return;
  }
  _robotPoseObject = 0;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  std::list<PoseObject*>::reverse_iterator it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (!_robotPoseObject) _robotPoseObject = *it;
    it++;
    count++;
  }
  if (!_robotPoseObject) return;
  _sensorPose = _robotPoseObject->vertex()->estimate() * _offsetParam->offset();
  for (const auto& it : world()->objects()) {
    WorldObjectType* o = dynamic_cast<WorldObjectType*>(it.get());
    if (o && isVisible(o)) {
      std::unique_ptr<EdgeType> e = mkEdge(o);
      e->setParameterId(0, _offsetParam->id());
      if (e && graph()) {
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
}
