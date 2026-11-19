// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/solvers/csparse/linear_solver_csparse.h>

G2O_START_NAMESPACE

// The one definition of the instantiation the header declares, which takes
// its visibility from that declaration.
template class LinearSolverCSparse<Eigen::MatrixXd>;

G2O_END_NAMESPACE
