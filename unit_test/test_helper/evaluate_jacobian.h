// SPDX-FileCopyrightText: 2014 R. Kuemmerle, G. Grisetti, W. Burgard
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef EVALUATE_JACOBIAN_H
#define EVALUATE_JACOBIAN_H

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <algorithm>
#include <cstddef>
#include <span>
#include <vector>

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"
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
  const std::span numeric(numericJacobianWorkspace.workspaceForVertex(0),
                          numElems);
  const std::span analytic(jacobianWorkspace.workspaceForVertex(0), numElems);
  EXPECT_THAT(analytic, ::testing::Pointwise(
                            ::testing::DoubleNear(JacobianTolerance), numeric));
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
    const int numElems = i == 0 ? numElemsXi : numElemsXj;
    const std::span numeric(numericJacobianWorkspace.workspaceForVertex(i),
                            numElems);
    const std::span analytic(jacobianWorkspace.workspaceForVertex(i), numElems);
    EXPECT_THAT(analytic,
                ::testing::Pointwise(::testing::DoubleNear(tolerance), numeric))
        << "Jacobian of vertex " << i;
  }
}

/**
 * Compares the analytic Jacobian of an edge with any number of vertices
 * against the numeric one of its base, vertex by vertex.
 */
template <typename EdgeType>
void evaluateJacobianMulti(EdgeType& e, JacobianWorkspace& jacobianWorkspace,
                           JacobianWorkspace& numericJacobianWorkspace,
                           double tolerance = JacobianTolerance) {
  using Base =
      BaseMultiEdge<EdgeType::Dimension, typename EdgeType::Measurement>;
  std::vector<int> numElems;
  for (std::size_t i = 0; i < e.vertices().size(); ++i) {
    const auto* v = static_cast<const OptimizableGraph::Vertex*>(e.vertex(i));
    numElems.push_back(EdgeType::Dimension * v->dimension());
    std::fill_n(numericJacobianWorkspace.workspaceForVertex(i), numElems.back(),
                UnwrittenJacobianEntry);
  }
  // calling the analytic Jacobian but writing to the numeric workspace
  e.Base::linearizeOplus(numericJacobianWorkspace);
  // copy result into analytic workspace
  jacobianWorkspace = numericJacobianWorkspace;

  // compute the numeric Jacobian into the numericJacobianWorkspace workspace as
  // setup by the previous call
  e.Base::linearizeOplus();

  for (std::size_t i = 0; i < numElems.size(); ++i) {
    const std::span numeric(numericJacobianWorkspace.workspaceForVertex(i),
                            numElems[i]);
    const std::span analytic(jacobianWorkspace.workspaceForVertex(i),
                             numElems[i]);
    EXPECT_THAT(analytic,
                ::testing::Pointwise(::testing::DoubleNear(tolerance), numeric))
        << "Jacobian of vertex " << i;
  }
}

}  // namespace g2o

#endif
