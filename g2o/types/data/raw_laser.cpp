// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/macros.h>
#include <g2o/stuff/read_values.h>
#include <g2o/types/data/raw_laser.h>

#include <cstddef>
#include <istream>
#include <limits>
#include <ostream>
#include <utility>
#include <vector>

using namespace std;

G2O_START_NAMESPACE

RawLaser::RawLaser()
    : RobotData(), _laserParams(0, 180, -M_PI / 2, M_PI / 180., 50., 0.1, 0) {}

RawLaser::~RawLaser() = default;

bool RawLaser::write(std::ostream& os) const {
  os << _laserParams.type << " " << _laserParams.firstBeamAngle << " "
     << _laserParams.fov << " " << _laserParams.angularStep << " "
     << _laserParams.maxRange << " " << _laserParams.accuracy << " "
     << _laserParams.remissionMode << " ";
  os << _ranges.size();
  for (double range : _ranges) {
    os << " " << range;
  }
  os << " " << _remissions.size();
  for (double remission : _remissions) {
    os << " " << remission;
  }
  os << FIXED(" " << timestamp() << " " << nameToken(hostname()) << " "
                  << loggerTimestamp());
  return os.good();
}

bool RawLaser::readScan(std::istream& is) {
  int type;
  double angle;
  double fov;
  double res;
  double maxrange;
  double acc;
  int remission_mode;
  is >> type >> angle >> fov >> res >> maxrange >> acc >> remission_mode;

  std::size_t beams = 0;
  std::vector<double> ranges;
  if (!readCount(is, beams, std::numeric_limits<int>::max()) ||
      !readValues(is, beams, ranges)) {
    return false;
  }
  std::size_t remissionCount = 0;
  std::vector<double> remissions;
  if (!readCount(is, remissionCount, std::numeric_limits<int>::max()) ||
      !readValues(is, remissionCount, remissions)) {
    return false;
  }
  _laserParams = LaserParameters(type, static_cast<int>(beams), angle, res,
                                 maxrange, acc, remission_mode);
  _ranges = std::move(ranges);
  _remissions = std::move(remissions);
  return true;
}

bool RawLaser::read(std::istream& is) {
  if (!readScan(is)) {
    return false;
  }

  // timestamp + host
  is >> _timestamp;
  is >> _hostname;
  is >> _loggerTimestamp;
  return !is.fail();
}

void RawLaser::setRanges(const vector<double>& ranges) { _ranges = ranges; }

void RawLaser::setRemissions(const std::vector<double>& remissions) {
  _remissions = remissions;
}

void RawLaser::setLaserParams(const LaserParameters& laserParams) {
  _laserParams = laserParams;
}

RawLaser::Point2DVector RawLaser::cartesian() const {
  Point2DVector points;
  for (size_t i = 0; i < _ranges.size(); ++i) {
    const double& r = _ranges[i];
    if (r < _laserParams.maxRange) {
      double alpha = _laserParams.firstBeamAngle + i * _laserParams.angularStep;
      points.emplace_back(cos(alpha) * r, sin(alpha) * r);
    }
  }
  return points;
}

G2O_END_NAMESPACE
