// SPDX-FileCopyrightText: 2016 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <utility>

using namespace g2o;
using namespace std;
using namespace Eigen;

G2O_USE_OPTIMIZATION_LIBRARY(csparse)

double uniform_rand(double lowerBndr, double upperBndr) {
  return lowerBndr +
         ((double)std::rand() / (RAND_MAX + 1.0)) * (upperBndr - lowerBndr);
}

double gauss_rand(double sigma) {
  double x, y, r2;
  do {
    x = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    y = -1.0 + 2.0 * uniform_rand(0.0, 1.0);
    r2 = x * x + y * y;
  } while (r2 > 1.0 || r2 == 0.0);
  return sigma * y * std::sqrt(-2.0 * log(r2) / r2);
}

Eigen::Isometry3d sample_noise_from_se3(const Vector6d& cov) {
  double nx = gauss_rand(cov(0));
  double ny = gauss_rand(cov(1));
  double nz = gauss_rand(cov(2));

  double nroll = gauss_rand(cov(3));
  double npitch = gauss_rand(cov(4));
  double nyaw = gauss_rand(cov(5));

  AngleAxisd aa(AngleAxisd(nyaw, Vector3d::UnitZ()) *
                AngleAxisd(nroll, Vector3d::UnitX()) *
                AngleAxisd(npitch, Vector3d::UnitY()));

  Eigen::Isometry3d retval = Isometry3d::Identity();
  retval.matrix().block<3, 3>(0, 0) = aa.toRotationMatrix();
  retval.translation() = Vector3d(nx, ny, nz);
  return retval;
}

Vector4d sample_noise_from_line(const Vector4d& cov) {
  return Vector4d(gauss_rand(cov(0)), gauss_rand(cov(1)), gauss_rand(cov(2)),
                  gauss_rand(cov(3)));
}

struct SimulatorItem {
  SimulatorItem(OptimizableGraph* graph_) : _graph(graph_) {}
  OptimizableGraph* graph() { return _graph; }
  virtual ~SimulatorItem() = default;

 protected:
  OptimizableGraph* _graph;
};

struct WorldItem : public SimulatorItem {
  WorldItem(OptimizableGraph* graph_,
            OptimizableGraph::Vertex* vertex_ = nullptr)
      : SimulatorItem(graph_), _vertex(vertex_) {}
  OptimizableGraph::Vertex* vertex() { return _vertex; }
  void setVertex(OptimizableGraph::Vertex* vertex_) { _vertex = vertex_; }

 protected:
  OptimizableGraph::Vertex* _vertex;
};

//! the items of the world, which the simulator owns
typedef std::set<std::unique_ptr<WorldItem>> WorldItemSet;

struct Robot;

struct Sensor {
  Sensor(Robot* robot_) : _robot(robot_) {}
  Robot* robot() { return _robot; }
  virtual bool isVisible(const WorldItem*) const { return false; }
  virtual bool sense(WorldItem*, const Isometry3d&) { return false; }
  virtual ~Sensor() = default;
  ;

 protected:
  Robot* _robot;
};

//! the sensors of a robot, which the robot owns
typedef std::vector<std::unique_ptr<Sensor>> SensorVector;

struct Robot : public WorldItem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Robot(OptimizableGraph* graph_) : WorldItem(graph_) {
    _planarMotion = false;
    _position = Isometry3d::Identity();
  }

  void move(const Isometry3d& newPosition, int& id) {
    Isometry3d delta = _position.inverse() * newPosition;
    _position = newPosition;
    auto vOwner = std::make_unique<VertexSE3>();
    auto* v = vOwner.get();
    v->setId(id);
    id++;
    graph()->addVertex(std::move(vOwner));
    if (_planarMotion) {
      // add a singleton constraint that locks the position of the robot on the
      // plane
      auto planeConstraint = std::make_unique<EdgeSE3Prior>();
      Matrix6d pinfo = Matrix6d::Zero();
      pinfo(2, 2) = 1e9;
      planeConstraint->setInformation(pinfo);
      planeConstraint->setMeasurement(Isometry3d::Identity());
      planeConstraint->vertices()[0] = v;
      planeConstraint->setParameterId(0, 0);
      graph()->addEdge(std::move(planeConstraint));
    }
    if (vertex() != nullptr) {
      VertexSE3* oldV = dynamic_cast<VertexSE3*>(vertex());
      auto eOwner = std::make_unique<EdgeSE3>();
      auto* e = eOwner.get();
      Isometry3d noise = sample_noise_from_se3(_nmovecov);
      e->setMeasurement(delta * noise);
      Matrix6d m = Matrix6d::Identity();
      for (int i = 0; i < 6; ++i) {
        m(i, i) = 1.0 / (_nmovecov(i));
      }
      e->setInformation(m);
      e->vertices()[0] = vertex();
      e->vertices()[1] = v;
      graph()->addEdge(std::move(eOwner));
      v->setEstimate(oldV->estimate() * e->measurement());
    } else {
      v->setEstimate(_position);
    }
    setVertex(v);
  }

  void relativeMove(const Isometry3d& delta, int& id) {
    Isometry3d newPosition = _position * delta;
    move(newPosition, id);
  }

  void sense(WorldItem* wi = nullptr) {
    for (const auto& s : _sensors) {
      s->sense(wi, _position);
    }
  }

  Isometry3d _position;
  SensorVector _sensors;
  Vector6d _nmovecov;
  bool _planarMotion;
};

//! the robots of the simulator, which the simulator owns
typedef std::vector<std::unique_ptr<Robot>> RobotVector;

struct Simulator : public SimulatorItem {
  Simulator(OptimizableGraph* graph_)
      : SimulatorItem(graph_), _lastVertexId(0) {}
  void sense(int robotIndex) {
    Robot* r = _robots[robotIndex].get();
    for (const auto& item : _world) {
      r->sense(item.get());
    }
  }

  void move(int robotIndex, const Isometry3d& newRobotPose) {
    Robot* r = _robots[robotIndex].get();
    r->move(newRobotPose, _lastVertexId);
  }

  void relativeMove(int robotIndex, const Isometry3d& delta) {
    Robot* r = _robots[robotIndex].get();
    r->relativeMove(delta, _lastVertexId);
  }

  int _lastVertexId;
  WorldItemSet _world;
  RobotVector _robots;
};

struct LineItem : public WorldItem {
  LineItem(OptimizableGraph* graph_, int id) : WorldItem(graph_) {
    auto lOwner = std::make_unique<VertexLine3D>();
    auto* l = lOwner.get();
    l->setId(id);
    graph()->addVertex(std::move(lOwner));
    setVertex(l);
  }
};

struct LineSensor : public Sensor {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LineSensor(Robot* r, int offsetId, const Isometry3d& offset_) : Sensor(r) {
    auto offsetVertex = std::make_unique<VertexSE3>();
    _offsetVertex = offsetVertex.get();
    _offsetVertex->setId(offsetId);
    _offsetVertex->setEstimate(offset_);
    robot()->graph()->addVertex(std::move(offsetVertex));
  };

  virtual bool isVisible(const WorldItem* wi) const {
    if (wi == nullptr) {
      return false;
    }
    const LineItem* li = dynamic_cast<const LineItem*>(wi);
    if (li == nullptr) {
      return false;
    }
    return true;
  }

  virtual bool sense(WorldItem* wi, const Isometry3d& position) {
    if (wi == nullptr) {
      return false;
    }
    LineItem* li = dynamic_cast<LineItem*>(wi);
    if (li == nullptr) {
      return false;
    }
    OptimizableGraph::Vertex* rv = robot()->vertex();
    if (rv == nullptr) {
      return false;
    }
    VertexSE3* robotVertex = dynamic_cast<VertexSE3*>(rv);
    if (robotVertex == nullptr) {
      return false;
    }
    const Isometry3d& robotPose = position;
    Isometry3d sensorPose = robotPose * _offsetVertex->estimate();
    VertexLine3D* lineVertex = dynamic_cast<VertexLine3D*>(li->vertex());
    Line3D worldLine = lineVertex->estimate();

    Line3D measuredLine = sensorPose.inverse() * worldLine;

    auto e = std::make_unique<EdgeSE3Line3D>();
    e->vertices()[0] = robotVertex;
    e->vertices()[1] = lineVertex;
    Vector4d noise = sample_noise_from_line(_nline);
    measuredLine.oplus(noise);
    e->setMeasurement(measuredLine);
    Matrix4d m = Matrix4d::Zero();
    m(0, 0) = 1.0 / (_nline(0));
    m(1, 1) = 1.0 / (_nline(1));
    m(2, 2) = 1.0 / (_nline(2));
    m(3, 3) = 1.0 / (_nline(3));
    e->setInformation(m);
    e->setParameterId(0, 0);
    robot()->graph()->addEdge(std::move(e));
    return true;
  }

  VertexSE3* _offsetVertex;
  Vector4d _nline;
};

int main(int argc, char** argv) {
  g2o::initializeLogging(argv[0]);
  bool verbose, robustKernel, fixLines, planarMotion, listSolvers;
  int maxIterations;
  double lambdaInit;
  string strSolver;
  CommandArgs arg;
  arg.param("i", maxIterations, 10, "perform n iterations");
  arg.param("v", verbose, false, "verbose output of the optimization process");
  arg.param("solver", strSolver, "lm_var", "select one specific solver");
  arg.param("lambdaInit", lambdaInit, 0,
            "user specified lambda init for levenberg");
  arg.param("robustKernel", robustKernel, false, "use robust error functions");
  arg.param("fixLines", fixLines, false,
            "fix the lines (do localization only)");
  arg.param("planarMotion", planarMotion, false, "robot moves on a plane");
  arg.param("listSolvers", listSolvers, false, "list the solvers");
  arg.parseArgs(argc, argv);

  auto g = std::make_unique<SparseOptimizer>();
  auto odomOffset = std::make_unique<ParameterSE3Offset>();
  odomOffset->setId(0);
  g->addParameter(std::move(odomOffset));

  OptimizationAlgorithmFactory* solverFactory =
      OptimizationAlgorithmFactory::instance();
  OptimizationAlgorithmProperty solverProperty;
  auto solver = solverFactory->construct(strSolver, solverProperty);
  g->setAlgorithm(std::move(solver));
  if (listSolvers) {
    solverFactory->listSolvers(std::cout);
    return 0;
  }

  if (g->solver() == nullptr) {
    ngx::Log(ngx::Error, "Error allocating solver. Allocating \"{}\" failed!",
             strSolver);
    std::cout << "Available solvers: " << std::endl;
    solverFactory->listSolvers(std::cout);
    std::cout << "--------------" << std::endl;
    return 0;
  }

  ngx::Log("Creating simulator");
  auto sim = std::make_unique<Simulator>(g.get());

  ngx::Log("Creating robot");
  auto robot = std::make_unique<Robot>(g.get());
  Robot* r = robot.get();

  ngx::Log("Creating line sensor");
  Isometry3d sensorPose = Isometry3d::Identity();
  auto lsOwner = std::make_unique<LineSensor>(r, 0, sensorPose);
  LineSensor* ls = lsOwner.get();
  ls->_nline << 0.001, 0.001, 0.001, 0.0001;
  // ls->_nline << 1e-9, 1e-9, 1e-9, 1e-9;
  r->_sensors.push_back(std::move(lsOwner));
  sim->_robots.push_back(std::move(robot));

  Line3D line;
  ngx::Log("Creating landmark line 1");
  auto liOwner = std::make_unique<LineItem>(g.get(), 1);
  LineItem* li = liOwner.get();
  Vector6d liv;
  liv << 0.0, 0.0, 5.0, 0.0, 1.0, 0.0;
  line = Line3D::fromCartesian(liv);
  dynamic_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(std::move(liOwner));

  ngx::Log("Creating landmark line 2");
  liv << 5.0, 0.0, 0.0, 0.0, 0.0, 1.0;
  line = Line3D::fromCartesian(liv);
  liOwner = std::make_unique<LineItem>(g.get(), 2);
  li = liOwner.get();
  dynamic_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(std::move(liOwner));

  ngx::Log("Creating landmark line 3");
  liv << 0.0, 5.0, 0.0, 1.0, 0.0, 0.0;
  line = Line3D::fromCartesian(liv);
  liOwner = std::make_unique<LineItem>(g.get(), 3);
  li = liOwner.get();
  dynamic_cast<VertexLine3D*>(li->vertex())->setEstimate(line);
  li->vertex()->setFixed(fixLines);
  sim->_world.insert(std::move(liOwner));

  Quaterniond q, iq;
  if (planarMotion) {
    r->_planarMotion = true;
    r->_nmovecov << 0.01, 0.0025, 1e-9, 0.001, 0.001, 0.025;
    q = Quaterniond(AngleAxisd(0.2, Vector3d::UnitZ()).toRotationMatrix());
    iq = Quaterniond(AngleAxisd(-0.2, Vector3d::UnitZ()).toRotationMatrix());
  } else {
    r->_planarMotion = false;
    r->_nmovecov << 0.1, 0.005, 1e-9, 0.001, 0.001, 0.05;
    q = Quaterniond((AngleAxisd(M_PI / 10, Vector3d::UnitZ()) *
                     AngleAxisd(0.1, Vector3d::UnitY()))
                        .toRotationMatrix());
    iq = Quaterniond((AngleAxisd(-M_PI / 10, Vector3d::UnitZ()) *
                      AngleAxisd(0.1, Vector3d::UnitY()))
                         .toRotationMatrix());
  }

  Isometry3d delta = Isometry3d::Identity();
  sim->_lastVertexId = 4;

  Isometry3d startPose = Isometry3d::Identity();
  startPose.matrix().block<3, 3>(0, 0) =
      AngleAxisd(-0.75 * M_PI, Vector3d::UnitZ()).toRotationMatrix();
  sim->move(0, startPose);

  int k = 20;
  int l = 2;
  double delta_t = 0.2;
  for (int j = 0; j < l; ++j) {
    Vector3d tr(1.0, 0.0, 0.0);
    delta.matrix().block<3, 3>(0, 0) = q.toRotationMatrix();
    if (j == l - 1) {
      delta.matrix().block<3, 3>(0, 0) = Matrix3d::Identity();
    }
    delta.translation() = tr * (delta_t * j);
    Isometry3d iDelta = delta.inverse();
    for (int a = 0; a < 2; ++a) {
      for (int i = 0; i < k; ++i) {
        if (a == 0) {
          sim->relativeMove(0, delta);
        } else {
          sim->relativeMove(0, iDelta);
        }
        sim->sense(0);
      }
    }
  }
  for (int j = 0; j < l; ++j) {
    Vector3d tr(1.0, 0.0, 0.0);
    delta.matrix().block<3, 3>(0, 0) = iq.toRotationMatrix();
    if (j == l - 1) {
      delta.matrix().block<3, 3>(0, 0) = Matrix3d::Identity();
    }
    delta.translation() = tr * (delta_t * j);
    Isometry3d iDelta = delta.inverse();
    for (int a = 0; a < 2; ++a) {
      for (int i = 0; i < k; ++i) {
        if (a == 0) {
          sim->relativeMove(0, delta);
        } else {
          sim->relativeMove(0, iDelta);
        }
        sim->sense(0);
      }
    }
  }

  ls->_offsetVertex->setFixed(true);
  OptimizableGraph::Vertex* gauge = g->vertex(4);
  if (gauge != nullptr) {
    gauge->setFixed(true);
  }

  ofstream osp("line3d.g2o");
  g->save(osp);
  ngx::Log("Saved graph on file line3d.g2o, use g2o_viewer to work with it.");

  return 0;
}
