// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "sensor_pose3d_offset.h"

#include <g2o/types/slam3d/isometry3d_mappings.h>

G2O_START_NAMESPACE
  using namespace std;
  using namespace Eigen;

  SensorPose3DOffset::SensorPose3DOffset(const std::string& name_): 
    BinarySensor<Robot3D, EdgeSE3Offset, WorldObjectSE3>(name_){  
    _offsetParam1 = _offsetParam2 =nullptr;
    _stepsToIgnore = 10;
    _information.setIdentity();
    _information*=100;
    _information(3,3)=10000;
    _information(4,4)=10000;
    _information(5,5)=1000;
    setInformation(_information);
  }

  void SensorPose3DOffset::addParameters(){
    if (_offsetParam1 == nullptr)
      _offsetParam1 = new ParameterSE3Offset();
    if (_offsetParam2 == nullptr)
      _offsetParam2 = new ParameterSE3Offset();
    assert(world());
    world()->addParameter(_offsetParam1);
    world()->addParameter(_offsetParam2);
  }

  void SensorPose3DOffset::addNoise(EdgeType* e){
    EdgeType::ErrorVector noise=_sampler.generateSample();
    EdgeType::Measurement n = internal::fromVectorMQT(noise);
    e->setMeasurement(e->measurement()*n);
    e->setInformation(information());
  }

  bool SensorPose3DOffset::isVisible(SensorPose3DOffset::WorldObjectType* to){
    if (_robotPoseObject == nullptr)
      return false;
    if (_posesToIgnore.find(to)!=_posesToIgnore.end())
      return false;
    
    assert(to && to->vertex());
    VertexType* v=to->vertex();
    VertexType::EstimateType pose=v->estimate();
    VertexType::EstimateType delta = _robotPoseObject->vertex()->estimate().inverse()*pose;
    Vector3d translation=delta.translation();
    double range2=translation.squaredNorm();
    if (range2>_maxRange2)
      return false;
    if (range2<_minRange2)
      return false;
    translation.normalize();
    double bearing=acos(translation.x());
    if (fabs(bearing)>_fov)
      return false;
    AngleAxisd a(delta.rotation());
    if (fabs(a.angle())>_maxAngularDifference)
      return false;
    return true;
  }
  
 
  void SensorPose3DOffset::sense() {
    _robotPoseObject=nullptr;
    RobotType* r= dynamic_cast<RobotType*>(robot());
    auto it=r->trajectory().rbegin();
    _posesToIgnore.clear();
    int count = 0;
    while (it!=r->trajectory().rend() && count < _stepsToIgnore){
      if (_robotPoseObject == nullptr)
  _robotPoseObject = *it;
      _posesToIgnore.insert(*it);
      it++;
      count++;
    }
    for (auto it : world()->objects()){
      WorldObjectType* o=dynamic_cast<WorldObjectType*>(it);
      if ((o != nullptr) && isVisible(o)){
  EdgeType* e=mkEdge(o);  
  if ((e != nullptr) && (graph() != nullptr)) {
          e->setParameterId(0,_offsetParam1->id());
          e->setParameterId(1,_offsetParam2->id());
    graph()->addEdge(e);
    e->setMeasurementFromState();
          addNoise(e);
  }
      }
    }
  }

G2O_END_NAMESPACE
