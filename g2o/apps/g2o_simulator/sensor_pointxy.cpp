// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_pointxy.h"

using namespace Eigen;

G2O_START_NAMESPACE

SensorPointXY::SensorPointXY(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2PointXY, WorldObjectPointXY>(name_) {}

void SensorPointXY::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

bool SensorPointXY::isVisible(SensorPointXY::WorldObjectType* to) {
  if (_robotPoseObject == nullptr) return false;

  assert(to && to->vertex());
  VertexType* v = to->vertex();
  VertexType::EstimateType pose = v->estimate();
  VertexType::EstimateType delta =
      _robotPoseObject->vertex()->estimate().inverse() * pose;
  Vector2d translation = delta;
  double range2 = translation.squaredNorm();
  if (range2 > _maxRange2) return false;
  if (range2 < _minRange2) return false;
  translation.normalize();
  double bearing = acos(translation.x());
  if (fabs(bearing) > _fov) return false;
  return true;
}

void SensorPointXY::sense() {
  _robotPoseObject = nullptr;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (_robotPoseObject == nullptr) _robotPoseObject = *it;
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
