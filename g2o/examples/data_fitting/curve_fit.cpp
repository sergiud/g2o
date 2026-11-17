// SPDX-FileCopyrightText: 2012 R. Kümmerle
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/sampler.h>

#include <Eigen/Core>
#include <iostream>

using namespace std;

/**
 * \brief the params, a, b, and lambda for a * exp(-lambda * t) + b
 */
class VertexParams : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexParams() {}

  virtual bool read(std::istream& /*is*/) {
    g2o::ngx::Log(g2o::ngx::Error, "not implemented yet");
    return false;
  }

  virtual bool write(std::ostream& /*os*/) const {
    g2o::ngx::Log(g2o::ngx::Error, "not implemented yet");
    return false;
  }

  virtual void setToOriginImpl() {
    g2o::ngx::Log(g2o::ngx::Error, "not implemented yet");
  }

  virtual void oplusImpl(const double* update) {
    Eigen::Vector3d::ConstMapType v(update);
    _estimate += v;
  }
};

/**
 * \brief measurement for a point on the curve
 *
 * Here the measurement is the point which is lies on the curve.
 * The error function computes the difference between the curve
 * and the point.
 */
class EdgePointOnCurve
    : public g2o::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgePointOnCurve() {}
  virtual bool read(std::istream& /*is*/) {
    g2o::ngx::Log(g2o::ngx::Error, "not implemented yet");
    return false;
  }
  virtual bool write(std::ostream& /*os*/) const {
    g2o::ngx::Log(g2o::ngx::Error, "not implemented yet");
    return false;
  }

  void computeError() {
    const VertexParams* params = dynamic_cast<const VertexParams*>(vertex(0));
    const double& a = params->estimate()(0);
    const double& b = params->estimate()(1);
    const double& lambda = params->estimate()(2);
    double fval = a * exp(-lambda * measurement()(0)) + b;
    _error(0) = fval - measurement()(1);
  }
};

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  int numPoints;
  int maxIterations;
  bool verbose;
  std::vector<int> gaugeList;
  string dumpFilename;
  g2o::CommandArgs arg;
  arg.param("dump", dumpFilename, "", "dump the points into a file");
  arg.param("numPoints", numPoints, 50,
            "number of points sampled from the curve");
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");

  arg.parseArgs(argc, argv);

  // generate random data
  double a = 2.;
  double b = 0.4;
  double lambda = 0.2;
  auto* points = new Eigen::Vector2d[numPoints];
  for (int i = 0; i < numPoints; ++i) {
    double x = g2o::Sampler::uniformRand(0, 10);
    double y = a * exp(-lambda * x) + b;
    // add Gaussian noise
    y += g2o::Sampler::gaussRand(0, 0.02);
    points[i].x() = x;
    points[i].y() = y;
  }

  if (!dumpFilename.empty()) {
    ofstream fout(dumpFilename.c_str());
    for (int i = 0; i < numPoints; ++i) fout << points[i].transpose() << endl;
  }

  // some handy typedefs
  typedef g2o::BlockSolver<
      g2o::BlockSolverTraits<Eigen::Dynamic, Eigen::Dynamic> >
      MyBlockSolver;
  typedef g2o::LinearSolverDense<MyBlockSolver::PoseMatrixType> MyLinearSolver;

  // setup the solver
  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  auto* linearSolver = new MyLinearSolver();
  auto* solver_ptr = new MyBlockSolver(linearSolver);
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
  optimizer.setAlgorithm(solver);

  // build the optimization problem given the points
  // 1. add the parameter vertex
  auto* params = new VertexParams();
  params->setId(0);
  params->setEstimate(
      Eigen::Vector3d(1, 1, 1));  // some initial value for the params
  optimizer.addVertex(params);
  // 2. add the points we measured to be on the curve
  for (int i = 0; i < numPoints; ++i) {
    auto* e = new EdgePointOnCurve;
    e->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
    e->setVertex(0, params);
    e->setMeasurement(points[i]);
    optimizer.addEdge(e);
  }

  // perform the optimization
  optimizer.initializeOptimization();
  optimizer.setVerbose(verbose);
  optimizer.optimize(maxIterations);

  if (verbose) cout << endl;

  // print out the result
  cout << "Target curve" << endl;
  cout << "a * exp(-lambda * x) + b" << endl;
  cout << "Iterative least squares solution" << endl;
  cout << "a      = " << params->estimate()(0) << endl;
  cout << "b      = " << params->estimate()(1) << endl;
  cout << "lambda = " << params->estimate()(2) << endl;
  cout << endl;

  // clean up
  delete[] points;

  return 0;
}
