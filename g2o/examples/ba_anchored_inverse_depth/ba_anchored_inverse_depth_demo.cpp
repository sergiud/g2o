// SPDX-FileCopyrightText: 2012 H. Strasdat
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/StdVector>
#include <cstdint>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <utility>
// #include <g2o/math_groups/se3quat.h>
#include <g2o/solvers/structure_only/structure_only_solver.h>
#include <g2o/stuff/logger.h>

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

Vector2d project2d(const Vector3d& v) {
  Vector2d res;
  res(0) = v(0) / v(2);
  res(1) = v(1) / v(2);
  return res;
}

Vector3d unproject2d(const Vector2d& v) {
  Vector3d res;
  res(0) = v(0);
  res(1) = v(1);
  res(2) = 1;
  return res;
}

inline Vector3d invert_depth(const Vector3d& x) {
  return unproject2d(x.head<2>()) / x[2];
}

int main(int argc, const char* argv[]) {
  g2o::initializeLogging(argv[0]);
  if (argc < 2) {
    cout << endl;
    cout << "Please type: " << endl;
    cout << "ba_anchored_inverse_depth_demo [PIXEL_NOISE] [OUTLIER RATIO] "
            "[ROBUST_KERNEL] [SCHUR-TRICK]"
         << endl;
    cout << endl;
    cout << "PIXEL_NOISE: noise in image space (E.g.: 1)" << endl;
    cout << "OUTLIER_RATIO: probability of spurious observation  (default: 0.0)"
         << endl;
    cout << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)"
         << endl;
    cout << "SCHUR-TRICK: Use Schur-complement trick (0 or 1; default: 1==true)"
         << endl;
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

  bool SCHUR_TRICK = true;
  if (argc > 4) {
    SCHUR_TRICK = atoi(argv[4]) != 0;
  }

  g2o::ngx::Log("PIXEL_NOISE: {}", PIXEL_NOISE);
  g2o::ngx::Log("OUTLIER_RATIO: {}", OUTLIER_RATIO);
  g2o::ngx::Log("ROBUST_KERNEL: {}", ROBUST_KERNEL);
  g2o::ngx::Log("SCHUR-TRICK: {}", SCHUR_TRICK);

  g2o::SparseOptimizer optimizer;
  optimizer.setVerbose(false);
  std::unique_ptr<g2o::OptimizationAlgorithmLevenberg> solver;
  if (SCHUR_TRICK) {
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;

    linearSolver = std::make_unique<
        g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    auto solver_ptr =
        std::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver));
    solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
        std::move(solver_ptr));
  } else {
    std::unique_ptr<g2o::BlockSolverX::LinearSolverType> linearSolver;
    linearSolver = std::make_unique<
        g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    auto solver_ptr =
        std::make_unique<g2o::BlockSolverX>(std::move(linearSolver));
    solver = std::make_unique<g2o::OptimizationAlgorithmLevenberg>(
        std::move(solver_ptr));
  }

  optimizer.setAlgorithm(std::move(solver));

  vector<Vector3d> true_points;
  for (size_t i = 0; i < 500; ++i) {
    true_points.emplace_back((Sample::uniform() - 0.5) * 3,
                             Sample::uniform() - 0.5, Sample::uniform() + 3);
  }

  double focal_length = 1000.;
  Vector2d principal_point(320., 240.);

  vector<g2o::SE3Quat, aligned_allocator<g2o::SE3Quat>> true_poses;

  auto cam_paramsOwner = std::make_unique<g2o::CameraParameters>(
      focal_length, principal_point, 0.);
  auto* cam_params = cam_paramsOwner.get();
  cam_params->setId(0);

  if (!optimizer.addParameter(std::move(cam_paramsOwner))) {
    assert(false);
  }

  int vertex_id = 0;
  for (size_t i = 0; i < 15; ++i) {
    Vector3d trans(i * 0.04 - 1., 0, 0);
    Eigen::Quaterniond q;
    q.setIdentity();
    g2o::SE3Quat T_me_from_world(q, trans);
    auto v_se3 = std::make_unique<g2o::VertexSE3Expmap>();
    v_se3->setId(vertex_id);
    v_se3->setEstimate(T_me_from_world);

    if (i < 2) {
      v_se3->setFixed(true);
    }

    optimizer.addVertex(std::move(v_se3));
    true_poses.push_back(T_me_from_world);
    vertex_id++;
  }
  int point_id = vertex_id;
  int point_num = 0;
  double sum_diff2 = 0;

  unordered_map<int, int> pointid_2_trueid;
  unordered_map<int, int> pointid_2_anchorid;
  unordered_set<int> inliers;

  for (size_t i = 0; i < true_points.size(); ++i) {
    auto v_pOwner = std::make_unique<g2o::VertexSBAPointXYZ>();
    g2o::VertexSBAPointXYZ* v_p = v_pOwner.get();
    int num_obs = 0;
    for (auto& true_pose : true_poses) {
      Vector2d z = cam_params->cam_map(true_pose.map(true_points.at(i)));

      if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
        ++num_obs;
      }
    }

    int anchor_frame_id = -1;
    if (num_obs >= 2) {
      bool inlier = true;
      for (size_t j = 0; j < true_poses.size(); ++j) {
        Vector2d z =
            cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

        if (z[0] >= 0 && z[1] >= 0 && z[0] < 640 && z[1] < 480) {
          if (anchor_frame_id == -1) {
            anchor_frame_id = j;
            pointid_2_anchorid.insert(make_pair(point_id, anchor_frame_id));
            g2o::SE3Quat T_anchor_from_world = true_poses[anchor_frame_id];
            v_p->setId(point_id);
            if (SCHUR_TRICK) {
              v_p->setMarginalized(true);
            }
            Vector3d point_w =
                true_points.at(i) + Vector3d(Sample::gaussian(1),
                                             Sample::gaussian(1),
                                             Sample::gaussian(1));

            Vector3d point_anchor = T_anchor_from_world * point_w;
            v_p->setEstimate(invert_depth(point_anchor));
            optimizer.addVertex(std::move(v_pOwner));
          }

          double sam = Sample::uniform();
          if (sam < OUTLIER_RATIO) {
            z = Vector2d(Sample::uniform(0, 640), Sample::uniform(0, 480));
            inlier = false;
          }
          z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                        Sample::gaussian(PIXEL_NOISE));
          auto e = std::make_unique<g2o::EdgeProjectPSI2UV>();

          e->resize(3);
          e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
          e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                              optimizer.vertices().find(j)->second.get()));
          e->setVertex(
              2, dynamic_cast<g2o::OptimizableGraph::Vertex*>(
                     optimizer.vertices().find(anchor_frame_id)->second.get()));

          e->setMeasurement(z);
          e->information() = Matrix2d::Identity();

          if (ROBUST_KERNEL) {
            e->setRobustKernel(std::make_unique<g2o::RobustKernelHuber>());
          }

          e->setParameterId(0, 0);
          optimizer.addEdge(std::move(e));
        }
      }
      if (inlier) {
        inliers.insert(point_id);
        Vector3d diff = true_poses[anchor_frame_id].inverse() *
                            invert_depth(v_p->estimate()) -
                        true_points[i];

        sum_diff2 += diff.dot(diff);
      }
      pointid_2_trueid.insert(make_pair(point_id, i));
      ++point_id;
      ++point_num;
    }
  }
  optimizer.initializeOptimization();
  optimizer.setVerbose(true);
  g2o::ngx::Log("Performing full BA:");

  optimizer.optimize(10);

  cout << "Point error before optimisation (inliers only): "
       << sqrt(sum_diff2 / inliers.size()) << endl;
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
    int anchorframe_id = pointid_2_anchorid.find(it.first)->second;
    v_it = optimizer.vertices().find(anchorframe_id);
    if (v_it == optimizer.vertices().end()) {
      g2o::ngx::Log(g2o::ngx::Error, "Vertex {} not in graph!", it.first);
      exit(-1);
    }
    g2o::VertexSE3Expmap* v_anchor =
        dynamic_cast<g2o::VertexSE3Expmap*>(v_it->second.get());
    if (v_p == nullptr) {
      g2o::ngx::Log(g2o::ngx::Error, "Vertex {} is not a SE3Expmap!", it.first);
      exit(-1);
    }
    Vector3d diff =
        v_anchor->estimate().inverse() * invert_depth(v_p->estimate()) -
        true_points[it.second];
    if (inliers.find(it.first) == inliers.end()) continue;
    sum_diff2 += diff.dot(diff);
    ++point_num;
  }
  cout << "Point error after optimisation (inliers only): "
       << sqrt(sum_diff2 / point_num) << endl;
  cout << endl;
}
