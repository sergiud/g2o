// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_segment2d.h"

#include <g2o/apps/g2o_simulator/simutils.h>
#include <g2o/stuff/logger.h>

G2O_START_NAMESPACE
using namespace std;
using namespace Eigen;

SensorSegment2D::SensorSegment2D(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2Segment2D, WorldObjectSegment2D>(name_) {
  ngx::Log("I am the constructor");
}

void SensorSegment2D::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

bool SensorSegment2D::isVisible(SensorSegment2D::WorldObjectType* to) {
  if (_robotPoseObject == nullptr) return false;

  assert(to && to->vertex());
  VertexType* v = to->vertex();

  Vector2d p1, p2;
  SE2 iRobot = _robotPoseObject->vertex()->estimate().inverse();
  p1 = iRobot * v->estimateP1();
  p2 = iRobot * v->estimateP2();

  Vector3d vp1(p1.x(), p1.y(), 0.);
  Vector3d vp2(p2.x(), p2.y(), 0.);
  Vector3d cp = vp1.cross(vp2);  // visibility check
  if (cp[2] < 0) return false;

  int circleClip = clipSegmentCircle(p1, p2, sqrt(_maxRange2));
  bool clip1 = false, clip2 = false;
  switch (circleClip) {
    case -1:
      return false;
    case 0:
      clip1 = true;
      break;
    case 1:
      clip2 = true;
      break;
    case 3:
      clip1 = true;
      clip2 = true;
      break;
    default:;
  }

  int fovClip = clipSegmentFov(p1, p2, -_fov, +_fov);
  switch (fovClip) {
    case -1:
      return false;
    case 0:
      clip1 = true;
      break;
    case 1:
      clip2 = true;
      break;
    case 3:
      clip1 = true;
      clip2 = true;
      break;
    default:;
  }
  if (!clip1 &&
      !clip2) {  // only if both endpoints have not been clipped do something
    return true;
  }
  // if all these checks went well, we set the point
  return false;
}

void SensorSegment2D::sense() {
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
