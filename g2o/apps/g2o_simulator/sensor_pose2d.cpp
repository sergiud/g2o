// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_pose2d.h"

using namespace Eigen;

G2O_START_NAMESPACE
SensorPose2D::SensorPose2D(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2, WorldObjectSE2>(name_) {
  _stepsToIgnore = 10;
}

bool SensorPose2D::isVisible(SensorPose2D::WorldObjectType* to) {
  if (_robotPoseObject == nullptr) return false;
  if (_posesToIgnore.find(to) != _posesToIgnore.end()) return false;

  assert(to && to->vertex());
  VertexType* v = to->vertex();
  VertexType::EstimateType pose = v->estimate();
  VertexType::EstimateType delta =
      _robotPoseObject->vertex()->estimate().inverse() * pose;
  Vector2d translation = delta.translation();
  double range2 = translation.squaredNorm();
  if (range2 > _maxRange2) return false;
  if (range2 < _minRange2) return false;
  translation.normalize();
  double bearing = acos(translation.x());
  if (fabs(bearing) > _fov) return false;
  if (fabs(delta.rotation().angle()) > _maxAngularDifference) return false;
  return true;
}

void SensorPose2D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector noise = _sampler.generateSample();
  EdgeType::Measurement n;
  n.fromVector(noise);
  e->setMeasurement(e->measurement() * n);
  e->setInformation(information());
}

void SensorPose2D::sense() {
  _robotPoseObject = nullptr;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  _posesToIgnore.clear();
  int count = 0;
  while (it != r->trajectory().rend() && count < _stepsToIgnore) {
    if (_robotPoseObject == nullptr) _robotPoseObject = *it;
    _posesToIgnore.insert(*it);
    it++;
    count++;
  }
  for (auto it : world()->objects()) {
    WorldObjectType* o = dynamic_cast<WorldObjectType*>(it);
    if ((o != nullptr) && isVisible(o)) {
      EdgeType* e = mkEdge(o);
      if ((e != nullptr) && (graph() != nullptr)) {
        e->setMeasurementFromState();
        addNoise(e);
        graph()->addEdge(e);
      }
    }
  }
}

G2O_END_NAMESPACE
