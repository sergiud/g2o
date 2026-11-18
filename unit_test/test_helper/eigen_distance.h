// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_UNIT_TEST_TEST_HELPER_EIGEN_DISTANCE_H
#define G2O_UNIT_TEST_TEST_HELPER_EIGEN_DISTANCE_H

namespace g2o::test {

/**
 * The largest absolute difference between the coefficients of two matrices,
 * which serves as the distance of DistanceFrom for Eigen types.
 */
inline constexpr auto MaxAbsDifference = [](const auto& left,
                                            const auto& right) {
  return (left - right).cwiseAbs().maxCoeff();
};

}  // namespace g2o::test

#endif
