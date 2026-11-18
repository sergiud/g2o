// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H
#define G2O_OPTIMIZATION_ALGORITHM_DOGLEG_H

#include <g2o/core/block_solver.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/prefix.h>

#include <memory>

G2O_START_NAMESPACE

/**
 * \brief Implementation of Powell's Dogleg Algorithm
 */
class G2O_CORE_API OptimizationAlgorithmDogleg
    : public OptimizationAlgorithmWithHessian {
 public:
  /** \brief type of the step to take */
  enum { STEP_UNDEFINED, STEP_SD, STEP_GN, STEP_DL };

 public:
  /**
   * construct the Dogleg algorithm, which will use the given Solver for solving
   * the linearized system.
   */
  //! The algorithm owns the solver.
  explicit OptimizationAlgorithmDogleg(std::unique_ptr<BlockSolverBase> solver);
  virtual ~OptimizationAlgorithmDogleg();

  virtual SolverResult solve(int iteration, bool online = false);

  virtual void printVerbose(std::ostream& os) const;

  //! return the type of the last step taken by the algorithm
  int lastStep() const { return _lastStep; }
  //! return the diameter of the trust region
  double trustRegion() const { return _delta; }

  //! convert the type into an integer
  static const char* stepType2Str(int stepType);

 protected:
  // parameters
  Property<int>* _maxTrialsAfterFailure;
  Property<double>* _userDeltaInit;
  // damping to enforce positive definite matrix
  Property<double>* _initialLambda;
  Property<double>* _lamdbaFactor;

  VectorXD _hsd;        ///< steepest decent step
  VectorXD _hdl;        ///< final dogleg step
  VectorXD _auxVector;  ///< auxilary vector used to perform multiplications or
                        ///< other stuff

  double _currentLambda{};  ///< the damping factor to force positive definite
                            ///< matrix
  double _delta;            ///< trust region
  int _lastStep;            ///< type of the step taken by the algorithm
  bool _wasPDInAllIterations;  ///< the matrix we solve was positive definite in
                               ///< all iterations -> if not apply damping
  int _lastNumTries{};
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
