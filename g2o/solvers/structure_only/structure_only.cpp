// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/creators.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/stuff/macros.h>

#include <memory>

G2O_START_NAMESPACE

/**
 * helper function for allocating
 */
static std::unique_ptr<OptimizationAlgorithm> createSolver(
    const std::string& fullSolverName) {
  if (fullSolverName == "structure_only_2") {
    return std::make_unique<StructureOnlySolver<2>>();
  } else if (fullSolverName == "structure_only_3") {
    return std::make_unique<StructureOnlySolver<3>>();
  } else
    return nullptr;
}

class StructureOnlyCreator : public AbstractOptimizationAlgorithmCreator {
 public:
  StructureOnlyCreator(const OptimizationAlgorithmProperty& p)
      : AbstractOptimizationAlgorithmCreator(p) {}
  std::unique_ptr<OptimizationAlgorithm> construct() override {
    return createSolver(property().name);
  }
};

G2O_REGISTER_OPTIMIZATION_LIBRARY(structure_only);

G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    structure_only_2,
    std::make_unique<StructureOnlyCreator>(OptimizationAlgorithmProperty(
        "structure_only_2", "Optimize the landmark poses (2D)", "Eigen", true,
        3, 2)));
G2O_REGISTER_OPTIMIZATION_ALGORITHM(
    structure_only_3,
    std::make_unique<StructureOnlyCreator>(OptimizationAlgorithmProperty(
        "structure_only_3", "Optimize the landmark poses (3D)", "Eigen", true,
        6, 3)));

G2O_END_NAMESPACE
