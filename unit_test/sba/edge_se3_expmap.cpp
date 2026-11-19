// SPDX-FileCopyrightText: 2026 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <random>

#include "g2o/core/jacobian_workspace.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/evaluate_jacobian.h"

namespace {

//! Pose of random rotation and translation, drawn from the given engine.
g2o::SE3Quat randomPose(std::mt19937& engine) {
  std::normal_distribution<double> normal;
  const Eigen::Vector3d axis(normal(engine), normal(engine), normal(engine));
  const Eigen::Vector3d translation(normal(engine), normal(engine),
                                    normal(engine));
  return {Eigen::Quaterniond(Eigen::AngleAxisd(axis.norm(), axis.normalized())),
          translation};
}

}  // namespace

/**
 * The analytic Jacobian of the relative pose neglects the curvature of the
 * logarithm, which leaves it exact for a consistent measurement, where the
 * optimization converges.
 */
TEST(EdgeSE3Expmap, JacobianIsExactWithoutError) {
  constexpr int Trials = 100;
  std::mt19937 engine;
  g2o::VertexSE3Expmap first;
  g2o::VertexSE3Expmap second;
  g2o::EdgeSE3Expmap edge;
  edge.setVertex(0, &first);
  edge.setVertex(1, &second);
  edge.setInformation(g2o::EdgeSE3Expmap::InformationType::Identity());
  g2o::JacobianWorkspace jacobianWorkspace;
  g2o::JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&edge);
  numericJacobianWorkspace.allocate();

  for (int trial = 0; trial < Trials; ++trial) {
    first.setEstimate(randomPose(engine));
    second.setEstimate(randomPose(engine));
    edge.setMeasurement(second.estimate() * first.estimate().inverse());

    g2o::evaluateJacobian(edge, jacobianWorkspace, numericJacobianWorkspace);
  }
}
