// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/abi.h>
#include <g2o/examples/calibration_odom_laser/g2o_calibration_odom_laser_api.h>
#include <g2o/prefix.h>

#include <string>

G2O_START_NAMESPACE

class SparseOptimizer;
class DataQueue;

/**
 * \brief read / write gm2dl file into / out of a SparseOptimizer
 */
class G2O_CALIBRATION_ODOM_LASER_API Gm2dlIO {
 public:
  /**
   * the global ID we assign the laser pose vertex
   */
  static const int ID_LASERPOSE;
  static const int ID_ODOMCALIB;

  static bool readGm2dl(const std::string& filename, SparseOptimizer& optimizer,
                        bool overrideCovariances = false);

  static bool updateLaserData(SparseOptimizer& optimizer);

  static bool writeGm2dl(const std::string& filename,
                         const SparseOptimizer& optimizer);

  static int readRobotLaser(const std::string& filename, DataQueue& queue);
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>
