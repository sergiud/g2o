// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef EVALUATE_JACOBIAN_H
#define EVALUATE_JACOBIAN_H

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"

namespace g2o {

/**
 * The value the analytic Jacobian finds in its workspace, so that an entry it
 * leaves unwritten differs from the numeric Jacobian.
 */
inline constexpr double UnwrittenJacobianEntry = 12345.;

//! The largest difference between the analytic and the numeric Jacobian.
inline constexpr double JacobianTolerance = 1e-6;

template <typename EdgeType>
void evaluateJacobianUnary(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                           JacobianWorkspace& numericJacobianWorkspace) {
  int numElems = EdgeType::Dimension;
  numElems *= EdgeType::VertexXiType::Dimension;
  std::fill_n(numericJacobianWorkspace.workspaceForVertex(0), numElems,
              UnwrittenJacobianEntry);
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseUnaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                  typename EdgeType::VertexXiType>::
      linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseUnaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                  typename EdgeType::VertexXiType>::linearizeOplus();

  // compare the Jacobians
  double* n = numericJacobianWorkspace.workspaceForVertex(0);
  double* a = jacobianWorkspace.workspaceForVertex(0);
  for (int j = 0; j < numElems; ++j) {
    EXPECT_NEAR(n[j], a[j], JacobianTolerance);
  }
}

template <typename EdgeType>
void evaluateJacobian(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                      JacobianWorkspace& numericJacobianWorkspace,
                      double tolerance = JacobianTolerance) {
  const int numElemsXi =
      EdgeType::Dimension * EdgeType::VertexXiType::Dimension;
  const int numElemsXj =
      EdgeType::Dimension * EdgeType::VertexXjType::Dimension;
  std::fill_n(numericJacobianWorkspace.workspaceForVertex(0), numElemsXi,
              UnwrittenJacobianEntry);
  std::fill_n(numericJacobianWorkspace.workspaceForVertex(1), numElemsXj,
              UnwrittenJacobianEntry);
  // calling the analytic Jacobian but writing to the numeric workspace
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::
      linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.BaseBinaryEdge<EdgeType::Dimension, typename EdgeType::Measurement,
                   typename EdgeType::VertexXiType,
                   typename EdgeType::VertexXjType>::linearizeOplus();

  // compare the two Jacobians
  for (int i = 0; i < 2; ++i) {
    double* n = numericJacobianWorkspace.workspaceForVertex(i);
    double* a = jacobianWorkspace.workspaceForVertex(i);
    const int numElems = i == 0 ? numElemsXi : numElemsXj;
    for (int j = 0; j < numElems; ++j) {
      EXPECT_NEAR(n[j], a[j], tolerance);
    }
  }
}

}  // namespace g2o

#endif
