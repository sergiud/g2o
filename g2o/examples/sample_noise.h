// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EXAMPLES_SAMPLE_NOISE_H
#define G2O_EXAMPLES_SAMPLE_NOISE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <random>

namespace g2o::examples {

/**
 * Draws zero-mean Gaussian noise for each coefficient with the variance of
 * the same coefficient, which the simulators invert into the information of
 * their measurements.
 */
template <typename Derived, typename Engine>
typename Derived::PlainObject sampleNoise(
    const Eigen::MatrixBase<Derived>& variances, Engine& engine) {
  using std::sqrt;
  typename Derived::PlainObject noise(variances.rows(), variances.cols());
  for (Eigen::Index i = 0; i < variances.size(); ++i) {
    std::normal_distribution<double> distribution(0., sqrt(variances(i)));
    noise(i) = distribution(engine);
  }
  return noise;
}

/**
 * Draws a pose of Gaussian noise whose translation and roll, pitch and yaw
 * angles have the given variances.
 */
template <typename Engine>
Eigen::Isometry3d samplePoseNoise(const Eigen::Matrix<double, 6, 1>& variances,
                                  Engine& engine) {
  const Eigen::Matrix<double, 6, 1> noise = sampleNoise(variances, engine);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = (Eigen::AngleAxisd(noise(5), Eigen::Vector3d::UnitZ()) *
                   Eigen::AngleAxisd(noise(3), Eigen::Vector3d::UnitX()) *
                   Eigen::AngleAxisd(noise(4), Eigen::Vector3d::UnitY()))
                      .toRotationMatrix();
  pose.translation() = noise.head<3>();
  return pose;
}

}  // namespace g2o::examples

#endif
