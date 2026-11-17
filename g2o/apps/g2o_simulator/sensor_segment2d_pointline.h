// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SENSOR_SEGMENT2D_POINTLINE_H_
#define G2O_SENSOR_SEGMENT2D_POINTLINE_H_

#include "simulator2d.h"
#include <g2o/apps/g2o_simulator/pointsensorparameters.h>
#include <g2o/types/slam2d_addons/types_slam2d_addons.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  // sensor that senses segments, only if the extremas are visible
  class G2O_SIMULATOR_API SensorSegment2DPointLine: public PointSensorParameters, public BinarySensor<Robot2D, EdgeSE2Segment2DPointLine,WorldObjectSegment2D>{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SensorSegment2DPointLine(const std::string& name_);
    virtual void sense();
    virtual void addNoise(EdgeType* e);
  protected:
    bool isVisible(WorldObjectType* to);
    int _visiblePoint{};
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
