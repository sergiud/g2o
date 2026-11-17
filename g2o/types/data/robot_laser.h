// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_ROBOT_LASER_H
#define G2O_ROBOT_LASER_H

#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/types/data/raw_laser.h>
#include <g2o/types/data/robot_laser.h>

G2O_START_NAMESPACE

/**
 * \brief laser measurement obtained by a robot
 *
 * A laser measurement obtained by a robot. The measurement is equipped with a
 * pose of the robot at which the measurement was taken. The read/write function
 * correspond to the CARMEN logfile format.
 */
class G2O_TYPES_DATA_API RobotLaser : public RawLaser {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  RobotLaser();
  ~RobotLaser();

  virtual bool write(std::ostream& os) const;
  virtual bool read(std::istream& is);

  SE2 laserPose() const { return _odomPose * _laserParams.laserPose; }
  const SE2& odomPose() const { return _odomPose; }
  void setOdomPose(const SE2& odomPose);

 protected:
  SE2 _odomPose;
  //! velocities and safety distances of the robot.
  double _laserTv, _laserRv, _forwardSafetyDist, _sideSaftyDist, _turnAxis;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_DATA_API RobotLaserDrawAction : public DrawAction {
 public:
  RobotLaserDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  IntProperty* _beamsDownsampling{};
  FloatProperty* _pointSize{};
  FloatProperty* _maxRange{};
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
