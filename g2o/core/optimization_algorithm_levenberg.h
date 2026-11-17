// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SOLVER_LEVENBERG_H
#define G2O_SOLVER_LEVENBERG_H

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

/**
 * \brief Implementation of the Levenberg Algorithm
 */
class G2O_CORE_API OptimizationAlgorithmLevenberg
    : public OptimizationAlgorithmWithHessian {
 public:
  /**
   * construct the Levenberg algorithm, which will use the given Solver for
   * solving the linearized system.
   */
  explicit OptimizationAlgorithmLevenberg(Solver* solver);
  virtual ~OptimizationAlgorithmLevenberg();

  virtual SolverResult solve(int iteration, bool online = false);

  virtual void printVerbose(std::ostream& os) const;

  //! return the currently used damping factor
  double currentLambda() const { return _currentLambda; }

  //! the number of internal iteration if an update step increases chi^2 within
  //! Levenberg-Marquardt
  void setMaxTrialsAfterFailure(int max_trials);

  //! get the number of inner iterations for Levenberg-Marquardt
  int maxTrialsAfterFailure() const { return _maxTrialsAfterFailure->value(); }

  //! return the lambda set by the user, if < 0 the SparseOptimizer will compute
  //! the initial lambda
  double userLambdaInit() { return _userLambdaInit->value(); }
  //! specify the initial lambda used for the first iteraion, if not given the
  //! SparseOptimizer tries to compute a suitable value
  void setUserLambdaInit(double lambda);

  //! return the number of levenberg iterations performed in the last round
  int levenbergIteration() { return _levenbergIterations; }

 protected:
  // Levenberg parameters
  Property<int>* _maxTrialsAfterFailure;
  Property<double>* _userLambdaInit;
  double _currentLambda;
  double _tau;
  double _goodStepLowerScale;  ///< lower bound for lambda decrease if a good LM
                               ///< step
  double _goodStepUpperScale;  ///< upper bound for lambda decrease if a good LM
                               ///< step
  double _ni;
  int _levenbergIterations;  ///< the numer of levenberg iterations performed to
                             ///< accept the last step

  /**
   * helper for Levenberg, this function computes the initial damping factor, if
   * the user did not specify an own value, see setUserLambdaInit()
   */
  double computeLambdaInit() const;
  double computeScale() const;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
