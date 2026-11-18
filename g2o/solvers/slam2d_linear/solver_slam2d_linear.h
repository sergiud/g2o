// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SOLVER_SLAM2D_LINEAR
#define G2O_SOLVER_SLAM2D_LINEAR

#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/prefix.h>
#include <g2o/solvers/slam2d_linear/g2o_slam2d_linear_api.h>

#include <memory>

G2O_START_NAMESPACE

class SparseOptimizer;
class Solver;

/**
 * \brief Implementation of a linear approximation for 2D pose graph SLAM
 *
 * Needs to operate on the full graph, whereas the nodes connected by
 * odometry are 0 -> 1 -> 2 -> ...
 * Furthermore excactly one node should be the fixed vertex.
 * Within the first iteration the orientation of the nodes is computed. In
 * the subsequent iterations full non-linear GN is carried out.
 * The linear approximation is correct, if the covariance of the constraints
 * is a diagonal matrix.
 *
 * More or less the solver is an implementation of the approach described
 * by Carlone et al, RSS'11.
 */
class G2O_SLAM2D_LINEAR_API SolverSLAM2DLinear
    : public OptimizationAlgorithmGaussNewton {
 public:
  /**
   * Construct a Solver for solving 2D pose graphs. Within the first iteration
   * the rotations are solved and afterwards standard non-linear Gauss Newton
   * is carried out.
   */
  //! The algorithm owns the solver.
  explicit SolverSLAM2DLinear(std::unique_ptr<Solver> solver);
  virtual ~SolverSLAM2DLinear();

  virtual OptimizationAlgorithm::SolverResult solve(int iteration,
                                                    bool online = false);

 protected:
  bool solveOrientation();
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
