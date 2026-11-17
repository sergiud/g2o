// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_RAW_LASER_H
#define G2O_RAW_LASER_H

#include <g2o/core/eigen_types.h>
#include <g2o/prefix.h>
#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/types/data/laser_parameters.h>
#include <g2o/types/data/robot_data.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

G2O_START_NAMESPACE

/**
 * \brief Raw laser measuerement
 *
 * A raw laser measuerement. The read/write function correspond to the format of
 * CARMEN.
 */
class G2O_TYPES_DATA_API RawLaser : public RobotData {
 public:
  typedef std::vector<Vector2D, Eigen::aligned_allocator<Vector2D> >
      Point2DVector;

 public:
  RawLaser();
  ~RawLaser();

  virtual bool write(std::ostream& os) const;
  virtual bool read(std::istream& is);

  /**
   * computes a cartesian view of the beams (x,y).
   * @return a vector with the points of the scan in cartesian coordinates.
   */
  Point2DVector cartesian() const;

  //! the range measurements by the laser
  const std::vector<double>& ranges() const { return _ranges; }
  void setRanges(const std::vector<double>& ranges);

  //! the remission measurements by the laser
  const std::vector<double>& remissions() const { return _remissions; }
  void setRemissions(const std::vector<double>& remissions);

  //! the parameters of the laser
  const LaserParameters& laserParams() const { return _laserParams; }
  void setLaserParams(const LaserParameters& laserParams);

 protected:
  std::vector<double> _ranges;
  std::vector<double> _remissions;
  LaserParameters _laserParams;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
