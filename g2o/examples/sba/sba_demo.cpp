// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/stuff/logger.h>
#include <g2o/types/icp/types_icp.h>

#include <Eigen/StdVector>
#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <utility>

#if defined G2O_HAVE_CHOLMOD
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#elif defined G2O_HAVE_CSPARSE
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#endif

using namespace Eigen;
using namespace std;

class Sample {
 public:
  static int uniform(int from, int to);
  static double uniform();
  static double gaussian(double sigma);
};

static double uniform_rand(double lowerBndr, double upperBndr) {
  return lowerBndr +
         ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

static double gauss_rand(double mean, double sigma) {
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return mean + sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

int Sample::uniform(int from, int to) {
  return static_cast<int>(uniform_rand(from, to));
}

double Sample::uniform() { return uniform_rand(0., 1.); }

double Sample::gaussian(double sigma) { return gauss_rand(0., sigma); }

int main(int argc, const char* argv[]) {
  g2o::initializeLogging(argv[0]);
  if (argc < 2) {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] "
            "[STRUCTURE_ONLY] [DENSE]"
         << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spuroius observation  (default: 0.0)"
         << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)"
         << endl;
    cout << "STRUCTURE_ONLY: performe structure-only BA to get better point "
            "initializations (0 or 1; default: 0==false)"
         << endl;
    cout << "DENSE: Use dense solver (0 or 1; default: 0==false)" << endl;
    cout << endl;
    cout << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to "
            "1==true."
         << endl;
    cout << endl;
    exit(0);
  }

  double PIXEL_NOISE = atof(argv[1]);

  double OUTLIER_RATIO = 0.0;

  if (argc > 2) {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc > 3) {
    ROBUST_KERNEL = atoi(argv[3]) != 0;
  }
  bool STRUCTURE_ONLY = false;
  if (argc > 4) {
    STRUCTURE_ONLY = atoi(argv[4]) != 0;
  }

  bool DENSE = false;
  if (argc > 5) {
    DENSE = atoi(argv[5]) != 0;
  }

  g2o::ngx::Log("PIXEL_NOISE: {}", PIXEL_NOISE);
  g2o::ngx::Log("OUTLIER_RATIO: {}", OUTLIER_RATIO);
  g2o::ngx::Log("ROBUST_KERNEL: {}", ROBUST_KERNEL);
  g2o::ngx::Log("STRUCTURE_ONLY: {}", STRUCTURE_ONLY);
  g2o::ngx::Log("DENSE: {}", DENSE);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
  if (DENSE) {
    linearSolver = std::make_unique<
        g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::ngx::Log("Using DENSE");
  } else {
#ifdef G2O_HAVE_CHOLMOD
    g2o::ngx::Log("Using CHOLMOD");
    linearSolver = std::make_unique<
        g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
#elif defined G2O_HAVE_CSPARSE
    linearSolver = std::make_unique<
        g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>>();
    g2o::ngx::Log("Using CSPARSE");
#else
#error neither CSparse nor Cholmod are available
#endif
  }

  auto solver_ptr =
      std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));

  auto solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
      std::move(solver_ptr));

  optimizer.setAlgorithm(std::move(solver));

  // set up 500 points
  vector<Vector3d> true_points;
  for (size_t i = 0; i < 500; ++i) {
    true_points.emplace_back((Sample::uniform() - 0.5) * 3,
                             Sample::uniform() - 0.5, Sample::uniform() + 10);
  }

  Vector2d focal_length(500, 500);     // pixels
  Vector2d principal_point(320, 240);  // 640x480 image
  double baseline = 0.075;             // 7.5 cm baseline

  vector<Eigen::Isometry3d, aligned_allocator<Eigen::Isometry3d>> true_poses;

  // set up camera params
  g2o::VertexSCam::setKcam(focal_length[0], focal_length[1], principal_point[0],
                           principal_point[1], baseline);

  // set up 5 vertices, first 2 fixed
  int vertex_id = 0;
  for (size_t i = 0; i < 5; ++i) {
    Vector3d trans(i * 0.04 - 1., 0, 0);

    Eigen::Quaterniond q;
    q.setIdentity();
    Eigen::Isometry3d pose;
    pose = q;
    pose.translation() = trans;

    auto v_se3 = std::make_unique<g2o::VertexSCam>();

    v_se3->setId(vertex_id);
    v_se3->setEstimate(pose);
    v_se3->setAll();  // set aux transforms

    if (i < 2) v_se3->setFixed(true);

    optimizer.addVertex(std::move(v_se3));
    true_poses.push_back(pose);
    vertex_id++;
  }

  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  unordered_map<int, int> pointid_2_trueid;
  unordered_set<int> inliers;

  // add point projections to this vertex
  for (size_t i = 0; i < true_points.size(); ++i) {
    auto v_pOwner = std::make_unique<g2o::VertexSBAPointXYZ>();
    auto* v_p = v_pOwner.get();

    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->setEstimate(true_points.at(i) + Vector3d(Sample::gaussian(1),
                                                  Sample::gaussian(1),
                                                  Sample::gaussian(1)));

    int num_obs = 0;

    for (size_t j = 0; j < true_poses.size(); ++j) {
      Vector3d z;
      dynamic_cast<g2o::VertexSCam*>(optimizer.vertices().find(j)->second.get())
          ->mapPoint(z, true_points.at(i));

      if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
        ++num_obs;
      }
    }

    if (num_obs >= 2) {
      optimizer.addVertex(std::move(v_pOwner));

      bool inlier = true;
      for (size_t j = 0; j < true_poses.size(); ++j) {
        Vector3d z;
        dynamic_cast<g2o::VertexSCam*>(
            optimizer.vertices().find(j)->second.get())
            ->mapPoint(z, true_points.at(i));

        if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
          double sam = Sample::uniform();
          if (sam < OUTLIER_RATIO) {
            z = Vector3d(Sample::uniform(64, 640), Sample::uniform(0, 480),
                         Sample::uniform(0, 64));  // disparity
            z(2) = z(0) - z(2);                    // px' now

            inlier = false;
          }

          z += Vector3d(Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE / 16.0));

          auto e = std::make_unique<g2o::Edge_XYZ_VSC>();

          e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

          e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(
              optimizer.vertices().find(j)->second.get());

          e->setMeasurement(z);
          // e->inverseMeasurement() = -z;
          e->information() = Matrix3d::Identity();

          if (ROBUST_KERNEL) {
            e->setRobustKernel(std::make_unique<g2o::RobustKernelHuber>());
          }

          optimizer.addEdge(std::move(e));
        }
      }

      if (inlier) {
        inliers.insert(point_id);
        Vector3d diff = v_p->estimate() - true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      // else
      //   cout << "Point: " << point_id <<  "has at least one spurious
      //   observation" <<endl;

      pointid_2_trueid.insert(make_pair(point_id, i));

      ++point_id;
      ++point_num;
    }
  }

  optimizer.initializeOptimization();

  optimizer.setVerbose(true);

  if (STRUCTURE_ONLY) {
    g2o::ngx::Log("Performing structure-only BA:");
    g2o::StructureOnlySolver<3> structure_only_ba;
    g2o::OptimizableGraph::VertexContainer points;
    for (g2o::OptimizableGraph::VertexIDMap::const_iterator it =
             optimizer.vertices().begin();
         it != optimizer.vertices().end(); ++it) {
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<g2o::OptimizableGraph::Vertex*>(it->second.get());
      if (v->dimension() == 3) points.push_back(v);
    }

    structure_only_ba.calc(points, 10);
  }

  g2o::ngx::Log("Performing full BA:");
  optimizer.optimize(10);

  cout << "Point error before optimisation (inliers only): "
       << sqrt(sum_diff2 / point_num) << endl;

  point_num = 0;
  sum_diff2 = 0;

  for (auto& it : pointid_2_trueid) {
    auto v_it = optimizer.vertices().find(it.first);

    if (v_it == optimizer.vertices().end()) {
      g2o::ngx::Log(g2o::ngx::Error, "Vertex {} not in graph!", it.first);
      exit(-1);
    }

    g2o::VertexSBAPointXYZ* v_p =
        dynamic_cast<g2o::VertexSBAPointXYZ*>(v_it->second.get());

    if (v_p == nullptr) {
      g2o::ngx::Log(g2o::ngx::Error, "Vertex {} is not a PointXYZ!", it.first);
      exit(-1);
    }

    Vector3d diff = v_p->estimate() - true_points[it.second];

    if (inliers.find(it.first) == inliers.end()) continue;

    sum_diff2 += diff.dot(diff);

    ++point_num;
  }

  cout << "Point error after optimisation (inliers only): "
       << sqrt(sum_diff2 / point_num) << endl;
  cout << endl;
}
