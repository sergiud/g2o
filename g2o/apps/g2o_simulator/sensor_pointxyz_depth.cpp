// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_pointxyz_depth.h"

G2O_START_NAMESPACE
  using namespace std;
  using namespace Eigen;

  // SensorPointXYZDepth
  SensorPointXYZDepth::SensorPointXYZDepth(const std::string& name_): BinarySensor<Robot3D, EdgeSE3PointXYZDepth, WorldObjectTrackXYZ>(name_) {
    _offsetParam = nullptr;
    _information.setIdentity();
    _information*=1000;
    setInformation(_information);
  }

  bool SensorPointXYZDepth::isVisible(SensorPointXYZDepth::WorldObjectType* to){
    if (_robotPoseObject == nullptr)
      return false;
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _sensorPose.inverse()*pose;
    Vector3d translation=delta;
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    // the cameras have the z in front
    double bearing=acos(translation.z());
    if (fabs(bearing)>_fov)
       return false;
    return true;
  }

  void SensorPointXYZDepth::addParameters(){
    if (_offsetParam == nullptr)
      _offsetParam = new ParameterCamera();
    assert(world());
    world()->addParameter(_offsetParam);
  }

  void SensorPointXYZDepth::addNoise(EdgeType* e){
    EdgeType::ErrorVector n=_sampler.generateSample();
    e->setMeasurement(e->measurement()+n);
    e->setInformation(information());
  }

  void SensorPointXYZDepth::sense() {
    if (_offsetParam == nullptr){
      return;
    }
    _robotPoseObject=nullptr;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    auto it=r->trajectory().rbegin();
    int count = 0;
    while (it!=r->trajectory().rend() && count < 1){
      if (_robotPoseObject == nullptr)
  _robotPoseObject = *it;
      it++;
      count++;
    }
    if (_robotPoseObject == nullptr)
      return;
    _sensorPose = _robotPoseObject->vertex()->estimate()*_offsetParam->offset();
    for (auto it : world()->objects()){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(it);
      if ((o != nullptr) && isVisible(o)){
  EdgeType* e=mkEdge(o);
  e->setParameterId(0,_offsetParam->id());
  if ((e != nullptr) && (graph() != nullptr)) {
    graph()->addEdge(e);
    e->setMeasurementFromState();
          addNoise(e);
  }
      }
    }
  }

G2O_END_NAMESPACE
