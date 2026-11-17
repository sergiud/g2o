// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H
#define G2O_OPTIMIZATION_ALGORITHM_GAUSS_NEWTON_H

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

/**
 * \brief Implementation of the Gauss Newton Algorithm
 */
class G2O_CORE_API OptimizationAlgorithmGaussNewton
    : public OptimizationAlgorithmWithHessian {
 public:
  /**
   * construct the Gauss Newton algorithm, which use the given Solver for
   * solving the linearized system.
   */
  explicit OptimizationAlgorithmGaussNewton(Solver* solver);
  virtual ~OptimizationAlgorithmGaussNewton();

  virtual SolverResult solve(int iteration, bool online = false);

  virtual void printVerbose(std::ostream& os) const;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
