// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/examples/sample_noise.h"

#include <Eigen/Core>
#include <random>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;

/**
 * The simulators weight their measurements with the inverse of the variances
 * they sample the noise with, so the samples must have these variances.
 */
TEST(SampleNoise, HasTheGivenVariances) {
  constexpr int Samples = 200000;
  // The variance of the sample variance is 2 sigma^4 / n, so this is about 10
  // standard deviations of the estimate.
  constexpr double RelativeTolerance = 0.03;
  const Eigen::Vector4d variances(1e-3, 4e-3, 1e-2, 1e-4);
  std::mt19937 engine;
  Eigen::Vector4d sum = Eigen::Vector4d::Zero();
  for (int i = 0; i < Samples; ++i) {
    const Eigen::Vector4d noise = g2o::examples::sampleNoise(variances, engine);
    sum += noise.cwiseProduct(noise);
  }
  const Eigen::Vector4d estimate = sum / Samples;

  for (int i = 0; i < variances.size(); ++i) {
    EXPECT_THAT(estimate[i],
                DoubleNear(variances[i], RelativeTolerance * variances[i]))
        << "coefficient " << i;
  }
}
