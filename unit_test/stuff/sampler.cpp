// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "g2o/stuff/sampler.h"

#include <Eigen/Core>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using Sampler = g2o::GaussianSampler<Eigen::Vector3d, Eigen::Matrix3d>;

/**
 * Two samplers with the same seed draw the same samples, which a state
 * shared between their engines would break.
 */
TEST(GaussianSampler, ReproducesItsSeed) {
  constexpr int Seed = 42;
  Sampler first;
  Sampler second;
  first.setDistribution(Eigen::Matrix3d::Identity());
  second.setDistribution(Eigen::Matrix3d::Identity());
  first.seed(Seed);
  second.seed(Seed);

  EXPECT_EQ(first.generateSample(), second.generateSample());
}

/**
 * A covariance without variance in one direction samples nothing along it,
 * such as a planar motion without noise in height.
 */
TEST(GaussianSampler, SamplesASemidefiniteCovariance) {
  Sampler sampler;
  ASSERT_TRUE(sampler.setDistribution(
      Eigen::Vector3d(1., 1., 0.).asDiagonal().toDenseMatrix()));

  const Eigen::Vector3d sample = sampler.generateSample();

  EXPECT_TRUE(sample.allFinite()) << sample.transpose();
  EXPECT_EQ(sample.z(), 0.);
}

//! A covariance with a negative variance describes no distribution.
TEST(GaussianSampler, RejectsAnIndefiniteCovariance) {
  Sampler sampler;

  EXPECT_FALSE(sampler.setDistribution(
      Eigen::Vector3d(1., 1., -1.).asDiagonal().toDenseMatrix()));
}

/**
 * The zero variances of a covariance of rank one come out of its
 * eigendecomposition slightly negative, whose square root has no value.
 */
TEST(GaussianSampler, SamplesACovarianceOfRankOne) {
  constexpr int Samples = 100;
  const Eigen::Vector3d direction = Eigen::Vector3d(1., 2., 3.).normalized();
  Sampler sampler;
  ASSERT_TRUE(sampler.setDistribution(direction * direction.transpose()));

  for (int i = 0; i < Samples; ++i) {
    const Eigen::Vector3d sample = sampler.generateSample();
    ASSERT_TRUE(sample.allFinite()) << sample.transpose();
  }
}
