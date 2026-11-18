// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G.Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "gm2dl_io.h"

#include <g2o/core/factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/types/data/data_queue.h>
#include <g2o/types/data/robot_laser.h>
#include <g2o/types/sclam2d/edge_se2_sensor_calib.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <utility>

using namespace std;

G2O_START_NAMESPACE

const int Gm2dlIO::ID_LASERPOSE = numeric_limits<int>::max();
const int Gm2dlIO::ID_ODOMCALIB = numeric_limits<int>::max() - 1;

bool Gm2dlIO::readGm2dl(const std::string& filename, SparseOptimizer& optimizer,
                        bool overrideCovariances) {
  optimizer.clear();
  ifstream is(filename.c_str());
  if (!is.good()) return false;

  bool laserOffsetInitDone = false;
  auto laserOffsetOwner = std::make_unique<VertexSE2>();
  auto* laserOffset = laserOffsetOwner.get();
  // laserOffset->fixed() = true;
  laserOffset->setId(ID_LASERPOSE);
  if (!optimizer.addVertex(std::move(laserOffsetOwner))) {
    ngx::Log(ngx::Error, "Unable to add laser offset");
    return false;
  }

  // parse the GM2DL file an extract the vertices, edges, and the laser data
  stringstream currentLine;
  VertexSE2* previousVertex = nullptr;
  while (1) {
    int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    string tag;
    currentLine >> tag;
    if (tag == "VERTEX" || tag == "VERTEX2" || tag == "VERTEX_SE2") {
      int id;
      Eigen::Vector3d p;
      currentLine >> id >> p.x() >> p.y() >> p.z();
      // adding the robot pose
      auto vOwner = std::make_unique<VertexSE2>();
      auto* v = vOwner.get();
      v->setId(id);
      // cerr << "Read vertex id " << id << endl;
      if (!optimizer.addVertex(std::move(vOwner))) {
        ngx::Log("vertex {} is already in the graph, reassigning ", id);
        v = dynamic_cast<VertexSE2*>(optimizer.vertex(id));
        assert(v);
      }
      v->setEstimate(p);
      previousVertex = v;

    } else if (tag == "EDGE" || tag == "EDGE2" || tag == "EDGE_SE2") {
      if (!laserOffsetInitDone) {
        ngx::Log(ngx::Error, "need laser offset");
        return false;
      }
      int id1, id2;
      auto eOwner = std::make_unique<EdgeSE2SensorCalib>();
      auto* e = eOwner.get();
      Eigen::Vector3d p;
      Eigen::Matrix3d& m = e->information();
      currentLine >> id1 >> id2 >> p.x() >> p.y() >> p.z();
      if (overrideCovariances) {
        m = Eigen::Matrix3d::Identity();
      } else {
        if (tag == "EDGE_SE2")
          currentLine >> m(0, 0) >> m(0, 1) >> m(0, 2) >> m(1, 1) >> m(1, 2) >>
              m(2, 2);
        else  // old stupid order of the information matrix
          currentLine >> m(0, 0) >> m(0, 1) >> m(1, 1) >> m(2, 2) >> m(0, 2) >>
              m(1, 2);
        m(1, 0) = m(0, 1);
        m(2, 0) = m(0, 2);
        m(2, 1) = m(1, 2);
      }
      previousVertex = nullptr;
      VertexSE2* v1 = dynamic_cast<VertexSE2*>(optimizer.vertex(id1));
      VertexSE2* v2 = dynamic_cast<VertexSE2*>(optimizer.vertex(id2));
      if (v1 == nullptr) {
        ngx::Log(ngx::Error,
                 "vertex {} is not existing, cannot add edge ({},{})", id1, id1,
                 id2);
        continue;
      }
      if (v2 == nullptr) {
        ngx::Log(ngx::Error,
                 "vertex {} is not existing, cannot add edge ({},{})", id2, id1,
                 id2);
        continue;
      }

      // if (0)
      // if (abs(id1 - id2) != 1)
      // m *= 1e-6;

      // TODO transform measurement covariance by considering the laserOffset to
      // measurement between the lasers
      SE2 transf;
      transf.fromVector(p);
      e->setMeasurement(laserOffset->estimate().inverse() * transf *
                        laserOffset->estimate());
      // e->inverseMeasurement() = e->measurement().inverse();

      e->setVertex(0, v1);
      e->setVertex(1, v2);
      e->setVertex(2, laserOffset);
      if (!optimizer.addEdge(std::move(eOwner))) {
        ngx::Log(ngx::Error, "error in adding edge {},{}", id1, id2);
      }
      // cerr << PVAR(e->inverseMeasurement().toVector().transpose()) << endl;
      // cerr << PVAR(e->information()) << endl;

    } else if (tag == "ROBOTLASER1") {
      if (previousVertex != nullptr) {
        auto rl2 = std::make_unique<RobotLaser>();
        rl2->read(currentLine);
        if (!laserOffsetInitDone) {
          laserOffsetInitDone = true;
          // cerr << "g2o Laseroffset is " <<
          // rl2->laserParams().laserPose.toVector().transpose() << endl;
          laserOffset->setEstimate(rl2->laserParams().laserPose);
        }
        previousVertex->setUserData(std::move(rl2));
        previousVertex = nullptr;
      }
    }
  }

  return true;
}

bool Gm2dlIO::writeGm2dl(const std::string& filename,
                         const SparseOptimizer& optimizer) {
  ofstream fout(filename.c_str());
  if (!fout.good()) {
    return false;
  }
  Factory* factory = Factory::instance();

  for (const auto& it : optimizer.vertices()) {
    OptimizableGraph::Vertex* v =
        dynamic_cast<OptimizableGraph::Vertex*>(it.second.get());
    fout << "VERTEX2 " << v->id() << " ";
    v->write(fout);
    fout << endl;
    HyperGraph::Data* data = v->userData();
    if (data != nullptr) {  // writing the data via the factory
      string tag = factory->tag(data);
      if (!tag.empty()) {
        fout << tag << " ";
        data->write(fout);
        fout << endl;
      }
    }
  }

  OptimizableGraph::EdgeContainer edgesToSave;  // sorting edges to have them in
                                                // the order of insertion again
  for (const auto& it : optimizer.edges()) {
    const OptimizableGraph::Edge* e =
        dynamic_cast<const OptimizableGraph::Edge*>(it.get());
    edgesToSave.push_back(const_cast<OptimizableGraph::Edge*>(e));
  }
  sort(edgesToSave.begin(), edgesToSave.end(),
       OptimizableGraph::EdgeIDCompare());

  for (OptimizableGraph::EdgeContainer::const_iterator it = edgesToSave.begin();
       it != edgesToSave.end(); ++it) {
    OptimizableGraph::Edge* e = *it;
    EdgeSE2SensorCalib* calibEdge = dynamic_cast<EdgeSE2SensorCalib*>(e);
    if (calibEdge != nullptr) {
      // write back in the gm2dl format
      fout << "EDGE2 " << calibEdge->vertex(0)->id() << " "
           << calibEdge->vertex(1)->id();
      Eigen::Vector3d meas = calibEdge->measurement().toVector();
      fout << " " << meas.x() << " " << meas.y() << " " << meas.z();
      const Eigen::Matrix3d& m = calibEdge->information();
      fout << " " << m(0, 0) << " " << m(0, 1) << " " << m(1, 1) << " "
           << m(2, 2) << " " << m(0, 2) << " " << m(1, 2);
      fout << endl;
    } else {
      // cerr << "Strange Edge Type: " << factory->tag(e) << endl;
    }
  }

  return fout.good();
}

bool Gm2dlIO::updateLaserData(SparseOptimizer& optimizer) {
  VertexSE2* laserOffset =
      dynamic_cast<VertexSE2*>(optimizer.vertex(ID_LASERPOSE));
  if (laserOffset == nullptr) {
    ngx::Log(ngx::Error, "Laser offset not found");
    return false;
  }

  for (SparseOptimizer::VertexIDMap::const_iterator it =
           optimizer.vertices().begin();
       it != optimizer.vertices().end(); ++it) {
    VertexSE2* v = dynamic_cast<VertexSE2*>(it->second.get());
    if (v == nullptr) continue;
    if (v->id() == ID_LASERPOSE) continue;
    RobotLaser* robotLaser = dynamic_cast<RobotLaser*>(v->userData());
    if (robotLaser != nullptr) {  // writing the data via the factory
      robotLaser->setOdomPose(v->estimate());
      LaserParameters params = robotLaser->laserParams();
      params.laserPose = laserOffset->estimate();
      robotLaser->setLaserParams(params);
    }
  }
  return true;
}

int Gm2dlIO::readRobotLaser(const std::string& filename, DataQueue& queue) {
  ifstream is(filename.c_str());
  if (!is.good()) return 0;

  int cnt = 0;

  // parse the GM2DL file and extract the vertices, edges, and the laser data
  stringstream currentLine;
  while (1) {
    int bytesRead = readLine(is, currentLine);
    if (bytesRead == -1) break;
    string tag;
    currentLine >> tag;
    if (tag == "ROBOTLASER1") {
      auto rl2 = std::make_unique<RobotLaser>();
      rl2->read(currentLine);
      queue.add(std::move(rl2));
      cnt++;
    }
  }
  return cnt;
}

G2O_END_NAMESPACE
