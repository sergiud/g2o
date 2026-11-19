// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/macros.h>
#include <g2o/types/data/robot_laser.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <iomanip>
using namespace std;

G2O_START_NAMESPACE

RobotLaser::RobotLaser()
    : RawLaser(),
      _laserTv(0.),
      _laserRv(0.),
      _forwardSafetyDist(0.),
      _sideSaftyDist(0.),
      _turnAxis(0.) {}

RobotLaser::~RobotLaser() = default;

bool RobotLaser::read(std::istream& is) {
  if (!readScan(is)) {
    return false;
  }

  // special robot laser stuff
  double x, y, theta;
  is >> x >> y >> theta;
  SE2 lp(x, y, theta);
  // cerr << "x: " << x << " y:" << y << " th:" << theta << " ";
  is >> x >> y >> theta;
  // cerr << "x: " << x << " y:" << y << " th:" << theta;
  _odomPose = SE2(x, y, theta);
  _laserParams.laserPose = _odomPose.inverse() * lp;
  is >> _laserTv >> _laserRv >> _forwardSafetyDist >> _sideSaftyDist >>
      _turnAxis;

  // timestamp + host
  is >> _timestamp;
  is >> _hostname;
  is >> _loggerTimestamp;
  return !is.fail();
}

bool RobotLaser::write(std::ostream& os) const {
  os << _laserParams.type << " " << _laserParams.firstBeamAngle << " "
     << _laserParams.fov << " " << _laserParams.angularStep << " "
     << _laserParams.maxRange << " " << _laserParams.accuracy << " "
     << _laserParams.remissionMode << " ";
  os << ranges().size();
  for (double i : ranges()) os << " " << i;
  os << " " << _remissions.size();
  for (double _remission : _remissions) os << " " << _remission;

  // odometry pose
  Vector3D p = (_odomPose * _laserParams.laserPose).toVector();
  os << " " << p.x() << " " << p.y() << " " << p.z();
  p = _odomPose.toVector();
  os << " " << p.x() << " " << p.y() << " " << p.z();

  // crap values
  os << FIXED(" " << _laserTv << " " << _laserRv << " " << _forwardSafetyDist
                  << " " << _sideSaftyDist << " " << _turnAxis);
  os << FIXED(" " << timestamp() << " " << nameToken(hostname()) << " "
                  << loggerTimestamp());

  return os.good();
}

void RobotLaser::setOdomPose(const SE2& odomPose) { _odomPose = odomPose; }

#ifdef G2O_HAVE_OPENGL
RobotLaserDrawAction::RobotLaserDrawAction()
    : DrawAction(typeid(RobotLaser).name()) {}

bool RobotLaserDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams != nullptr) {
    _beamsDownsampling = _previousParams->makeProperty<IntProperty>(
        _typeName + "::BEAMS_DOWNSAMPLING", 1);
    _pointSize = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::POINT_SIZE", 1.0f);
    _maxRange = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::MAX_RANGE", -1.);
  } else {
    _beamsDownsampling = nullptr;
    _pointSize = nullptr;
    _maxRange = nullptr;
  }
  return true;
}

HyperGraphElementAction* RobotLaserDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;

  refreshPropertyPtrs(params_);
  if (_previousParams == nullptr) {
    return this;
  }
  if ((_show != nullptr) && !_show->value()) return this;
  RobotLaser* that = dynamic_cast<RobotLaser*>(element);

  RawLaser::Point2DVector points = that->cartesian();
  if ((_maxRange != nullptr) && _maxRange->value() >= 0) {
    // prune the cartesian points;
    RawLaser::Point2DVector npoints(points.size());
    int k = 0;
    float r2 = _maxRange->value();
    r2 *= r2;
    for (auto& point : points) {
      double x = point.x();
      double y = point.y();
      if (x * x + y * y < r2) npoints[k++] = point;
    }
    points = npoints;
    npoints.resize(k);
  }

  glPushMatrix();
  const SE2& laserPose = that->laserParams().laserPose;
  glTranslatef((float)laserPose.translation().x(),
               (float)laserPose.translation().y(), 0.f);
  glRotatef((float)RAD2DEG(laserPose.rotation().angle()), 0.f, 0.f, 1.f);
  glColor4f(1.f, 0.f, 0.f, 0.5f);
  int step = 1;
  if (_beamsDownsampling != nullptr) step = _beamsDownsampling->value();
  if (_pointSize != nullptr) {
    glPointSize(_pointSize->value());
  }

  glBegin(GL_POINTS);
  for (size_t i = 0; i < points.size(); i += step) {
    glVertex3f((float)points[i].x(), (float)points[i].y(), 0.f);
  }
  glEnd();
  glPopMatrix();

  return this;
}
#endif

G2O_END_NAMESPACE
