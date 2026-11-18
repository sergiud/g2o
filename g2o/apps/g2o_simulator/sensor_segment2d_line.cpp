// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_segment2d_line.h"

#include <g2o/apps/g2o_simulator/simutils.h>

#include <memory>
#include <utility>
using namespace std;
using namespace Eigen;

G2O_START_NAMESPACE

SensorSegment2DLine::SensorSegment2DLine(const std::string& name_)
    : BinarySensor<Robot2D, EdgeSE2Segment2DLine, WorldObjectSegment2D>(name_) {
}

void SensorSegment2DLine::addNoise(EdgeType* e) {
  EdgeType::ErrorVector n = _sampler.generateSample();
  e->setMeasurement(e->measurement() + n);
  e->setInformation(information());
}

bool SensorSegment2DLine::isVisible(SensorSegment2DLine::WorldObjectType* to) {
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
  if (clip1 && clip2)  // only if both endpoints have been clipped do something
    return true;
  // if all these checks went well, we set the point
  return false;
}

void SensorSegment2DLine::sense() {
  _robotPoseObject = nullptr;
  RobotType* r = dynamic_cast<RobotType*>(robot());
  auto it = r->trajectory().rbegin();
  int count = 0;
  while (it != r->trajectory().rend() && count < 1) {
    if (_robotPoseObject == nullptr) _robotPoseObject = *it;
    it++;
    count++;
  }
  for (const auto& it : world()->objects()) {
    WorldObjectType* o = dynamic_cast<WorldObjectType*>(it.get());
    if ((o != nullptr) && isVisible(o)) {
      std::unique_ptr<EdgeType> e = mkEdge(o);
      if ((e != nullptr) && (graph() != nullptr)) {
        e->setMeasurementFromState();
        addNoise(e.get());
        graph()->addEdge(std::move(e));
      }
    }
  }
}

G2O_END_NAMESPACE
