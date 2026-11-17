// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H
#define G2O_OPTIMIZATION_ALGORITHM_WITH_HESSIAN_H

#include <g2o/core/g2o_core_api.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/prefix.h>

G2O_START_NAMESPACE

class Solver;

/**
 * \brief Base for solvers operating on the approximated Hessian, e.g.,
 * Gauss-Newton, Levenberg
 */
class G2O_CORE_API OptimizationAlgorithmWithHessian
    : public OptimizationAlgorithm {
 public:
  explicit OptimizationAlgorithmWithHessian(Solver* solver);
  virtual ~OptimizationAlgorithmWithHessian();

  virtual bool init(bool online = false);

  virtual bool computeMarginals(
      SparseBlockMatrix<MatrixXD>& spinv,
      const std::vector<std::pair<int, int> >& blockIndices);

  virtual bool buildLinearStructure();

  virtual void updateLinearSystem();

  virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>& vset,
                               const HyperGraph::EdgeSet& edges);

  //! return the underlying solver used to solve the linear system
  Solver* solver() { return _solver; }

  /**
   * write debug output of the Hessian if system is not positive definite
   */
  virtual void setWriteDebug(bool writeDebug);
  virtual bool writeDebug() const { return _writeDebug->value(); }

 protected:
  Solver* _solver;
  Property<bool>* _writeDebug;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
