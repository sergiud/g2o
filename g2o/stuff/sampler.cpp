// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/stuff/sampler.h>

G2O_START_NAMESPACE

  static std::normal_distribution<double> _univariateSampler(0., 1.);
  static std::uniform_real_distribution<double> _uniformReal;
  static std::mt19937 _gen_real;
 
  double sampleUniform(double min, double max, std::mt19937* generator){
    if (generator != nullptr)
      return _uniformReal(*generator)*(max-min)+min;
    return _uniformReal(_gen_real)*(max-min)+min;
  }

  double sampleGaussian(std::mt19937* generator){
    if (generator != nullptr)
      return _univariateSampler(*generator);
    return _univariateSampler(_gen_real);
  }

G2O_END_NAMESPACE
