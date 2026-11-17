// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/isometry3d_gradients.h>

#include <iostream>
G2O_START_NAMESPACE
using namespace std;
using namespace Eigen;

namespace internal {
#include "dquat2mat.cpp"
}  // end namespace internal

G2O_END_NAMESPACE
