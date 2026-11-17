// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef EVALUATE_JACOBIAN_H
#define EVALUATE_JACOBIAN_H

#include <gtest/gtest.h>

#include <Eigen/Core>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_unary_edge.h"

namespace g2o {

template <typename EdgeType>
void evaluateJacobianUnary(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                           JacobianWorkspace& numericJacobianWorkspace) {
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
  int numElems = EdgeType::Dimension;
  numElems *= EdgeType::VertexXiType::Dimension;
  for (int j = 0; j < numElems; ++j) {
    EXPECT_NEAR(n[j], a[j], 1e-6);
  }
}

template <typename EdgeType>
void evaluateJacobian(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                      JacobianWorkspace& numericJacobianWorkspace) {
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
    int numElems = EdgeType::Dimension;
    if (i == 0)
      numElems *= EdgeType::VertexXiType::Dimension;
    else
      numElems *= EdgeType::VertexXjType::Dimension;
    for (int j = 0; j < numElems; ++j) {
      EXPECT_NEAR(n[j], a[j], 1e-6);
    }
  }
}

}  // namespace g2o

#endif
