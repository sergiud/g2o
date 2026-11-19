// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <cmath>
#include <memory>
#include <string>

#include "g2o/core/robust_kernel.h"
#include "g2o/core/robust_kernel_factory.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

using ::testing::DoubleNear;

namespace {

constexpr double Delta = 2.;

std::unique_ptr<g2o::RobustKernel> kernel(const std::string& name) {
  auto* creator = g2o::RobustKernelFactory::instance()->creator(name);
  EXPECT_NE(creator, nullptr) << name;
  std::unique_ptr<g2o::RobustKernel> k(creator->construct());
  k->setDelta(Delta);
  return k;
}

Eigen::Vector3d robustify(const g2o::RobustKernel& k, double e2) {
  Eigen::Vector3d rho;
  k.robustify(e2, rho);
  return rho;
}

class RobustKernel : public ::testing::TestWithParam<std::string> {};

}  // namespace

/**
 * Every kernel behaves as the squared error for small errors, so its value
 * divided by the squared error is one up to a term of the order of the error
 * or, for the Fair kernel, of its square root.
 */
TEST_P(RobustKernel, KeepsTheDigitsOfSmallErrors) {
  constexpr double SmallError = 1e-28;
  constexpr double Tolerance = 1e-12;
  const std::unique_ptr<g2o::RobustKernel> k = kernel(GetParam());

  EXPECT_THAT(robustify(*k, SmallError)[0] / SmallError,
              DoubleNear(1., Tolerance));
}

/**
 * The second and third components are the first and second derivatives of
 * the first with respect to the squared error, which central differences
 * approximate.
 */
TEST_P(RobustKernel, ReturnsItsDerivatives) {
  constexpr double Step = 1e-5;
  constexpr double RelativeTolerance = 1e-6;
  const std::unique_ptr<g2o::RobustKernel> k = kernel(GetParam());
  // An inlier and an outlier of the kernels with a threshold.
  for (const double e2 : {0.5, 9.}) {
    const Eigen::Vector3d rho = robustify(*k, e2);
    const Eigen::Vector3d below = robustify(*k, e2 - Step);
    const Eigen::Vector3d above = robustify(*k, e2 + Step);
    const double first = (above[0] - below[0]) / (2. * Step);
    const double second = (above[1] - below[1]) / (2. * Step);

    EXPECT_THAT(rho[1],
                DoubleNear(first, RelativeTolerance * (std::abs(first) + 1.)))
        << "squared error " << e2;
    EXPECT_THAT(rho[2],
                DoubleNear(second, RelativeTolerance * (std::abs(second) + 1.)))
        << "squared error " << e2;
  }
}

INSTANTIATE_TEST_SUITE_P(Kernels, RobustKernel,
                         ::testing::Values("Huber", "PseudoHuber", "Cauchy",
                                           "GemanMcClure", "Welsch", "Fair",
                                           "Tukey", "Saturated"));
