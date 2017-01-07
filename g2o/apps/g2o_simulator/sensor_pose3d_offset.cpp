// g2o - General Graph Optimization
// Copyright (C) 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "sensor_pose3d_offset.h"

#include <g2o/types/slam3d/isometry3d_mappings.h>

namespace g2o {
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

} // namespace g2o
