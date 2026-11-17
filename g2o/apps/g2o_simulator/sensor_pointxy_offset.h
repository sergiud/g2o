// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_POINTXY_OFFSET_H_
#define G2O_SENSOR_POINTXY_OFFSET_H_

#include "pointsensorparameters.h"
#include "sensor_segment2d.h"
#include "sensor_segment2d_line.h"

#include <g2o/prefix.h>

G2O_START_NAMESPACE
  
  class G2O_SIMULATOR_API SensorPointXYOffset: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2PointXYOffset, WorldObjectPointXY>{ 
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef PoseVertexType::EstimateType RobotPoseType;
    SensorPointXYOffset(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);
    virtual void addParameters();
  protected:
    bool isVisible(WorldObjectType* to);
    ParameterSE2Offset* _offsetParam;
    RobotPoseType _sensorPose;
  }; 

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
