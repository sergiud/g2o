// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/sampler.h>

G2O_START_NAMESPACE

namespace {

std::mt19937 _gen_real;

}  // namespace

// The distributions keep state between draws, such as the second variate of
// a normal distribution, which must not pass from one engine to another.
double sampleUniform(double min, double max, std::mt19937* generator) {
  std::uniform_real_distribution<double> uniform;
  if (generator != nullptr) return uniform(*generator) * (max - min) + min;
  return uniform(_gen_real) * (max - min) + min;
}

double sampleGaussian(std::mt19937* generator) {
  std::normal_distribution<double> normal(0., 1.);
  if (generator != nullptr) return normal(*generator);
  return normal(_gen_real);
}

G2O_END_NAMESPACE
