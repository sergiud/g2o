// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <random>

#include "g2o/types/slam2d/edge_pointxy.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/edge_se2_pointxy.h"
#include "g2o/types/slam2d/edge_se2_pointxy_bearing.h"
#include "g2o/types/slam2d/edge_se2_prior.h"
#include "gtest/gtest.h"
#include "unit_test/test_helper/evaluate_jacobian.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

static SE2 randomSE2() { return SE2(Vector3d::Random()); }

TEST(Slam2D, EdgeSE2Jacobian) {
  VertexSE2 v1;
  v1.setId(0);

  VertexSE2 v2;
  v2.setId(1);

  EdgeSE2 e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgeSE2::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1.setEstimate(randomSE2());
    v2.setEstimate(randomSE2());
    e.setMeasurement(randomSE2());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2Prior) {
  VertexSE2 v1;
  v1.setId(0);

  EdgeSE2Prior e;
  e.setVertex(0, &v1);
  e.setInformation(EdgeSE2::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1.setEstimate(randomSE2());
    e.setMeasurement(randomSE2());

    evaluateJacobianUnary(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgePointXY) {
  VertexPointXY v1;
  v1.setId(0);

  VertexPointXY v2;
  v2.setId(1);

  EdgePointXY e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgePointXY::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1.setEstimate(Eigen::Vector2d::Random());
    v2.setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(Eigen::Vector2d::Random());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2PointXY) {
  VertexSE2 v1;
  v1.setId(0);

  VertexPointXY v2;
  v2.setId(1);

  EdgeSE2PointXY e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgeSE2PointXY::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  for (int k = 0; k < 10000; ++k) {
    v1.setEstimate(randomSE2());
    v2.setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(Eigen::Vector2d::Random());

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}

TEST(Slam2D, EdgeSE2PointXYBearing) {
  VertexSE2 v1;
  v1.setId(0);

  VertexPointXY v2;
  v2.setId(1);

  EdgeSE2PointXYBearing e;
  e.setVertex(0, &v1);
  e.setVertex(1, &v2);
  e.setInformation(EdgeSE2PointXYBearing::InformationType::Identity());

  JacobianWorkspace jacobianWorkspace;
  JacobianWorkspace numericJacobianWorkspace;
  numericJacobianWorkspace.updateSize(&e);
  numericJacobianWorkspace.allocate();

  std::default_random_engine eng;
  std::uniform_real_distribution<> dist;

  for (int k = 0; k < 10000; ++k) {
    v1.setEstimate(randomSE2());
    v2.setEstimate(Eigen::Vector2d::Random());
    e.setMeasurement(dist(eng) * M_PI);

    evaluateJacobian(e, jacobianWorkspace, numericJacobianWorkspace);
  }
}
