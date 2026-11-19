// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_UNIT_TEST_TEST_HELPER_EIGEN_DISTANCE_H
#define G2O_UNIT_TEST_TEST_HELPER_EIGEN_DISTANCE_H

#include <Eigen/Core>

namespace g2o::test {

/**
 * The largest absolute difference between the coefficients of two matrices,
 * which serves as the distance of DistanceFrom for Eigen types. A NaN in
 * either matrix makes the distance NaN, which no bound accepts.
 */
inline constexpr auto MaxAbsDifference = [](const auto& left,
                                            const auto& right) {
  return (left - right).cwiseAbs().template maxCoeff<Eigen::PropagateNaN>();
};

}  // namespace g2o::test

#endif
