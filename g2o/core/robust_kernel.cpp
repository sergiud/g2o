// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/robust_kernel.h>

G2O_START_NAMESPACE

RobustKernel::RobustKernel() :
  _delta(1.)
{
}

RobustKernel::RobustKernel(double delta) :
  _delta(delta)
{
}

void RobustKernel::setDelta(double delta)
{
  _delta = delta;
}

G2O_END_NAMESPACE
