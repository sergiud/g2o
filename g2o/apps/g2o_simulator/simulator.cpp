// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "simulator.h"

#include <iostream>
#include <memory>
#include <utility>

G2O_START_NAMESPACE
using namespace std;

// BaseWorldObject
BaseWorldObject::~BaseWorldObject() = default;
OptimizableGraph* BaseWorldObject::graph() {
  if (_world != nullptr) return _world->graph();
  return nullptr;
}

void BaseWorldObject::setVertex(OptimizableGraph::Vertex* vertex_) {
  if (_ownedVertex.get() != vertex_) _ownedVertex.reset();
  _vertex = vertex_;
}

// BaseRobot
OptimizableGraph* BaseRobot::graph() {
  if (_world != nullptr) return _world->graph();
  return nullptr;
}

bool BaseRobot::addSensor(std::unique_ptr<BaseSensor> sensor) {
  assert(graph());
  BaseSensor* added = sensor.get();
  if (!_sensors.insert(std::move(sensor)).second) return false;
  added->setRobot(this);
  added->addParameters();
  return true;
}

void BaseRobot::sense() {
  for (const auto& s : _sensors) {
    s->sense();
  }
}

// Sensor
World* BaseSensor::world() {
  if (_robot == nullptr) return nullptr;
  return _robot->world();
}

OptimizableGraph* BaseSensor::graph() {
  if (_robot == nullptr) return nullptr;
  return _robot->graph();
}

// World
bool World::addRobot(BaseRobot* robot) {
  std::pair<std::set<BaseRobot*>::iterator, bool> result =
      _robots.insert(robot);
  if (result.second) {
    robot->setWorld(this);
  }
  return result.second;
}

bool World::addWorldObject(std::unique_ptr<BaseWorldObject> object) {
  BaseWorldObject* added = object.get();
  if (!_objects.insert(std::move(object)).second) return false;
  added->setWorld(this);
  if (graph() == nullptr) return true;
  // The graph receives only a vertex the object created, since the caller
  // keeps owning a vertex it set.
  std::unique_ptr<OptimizableGraph::Vertex> vertex = added->releaseVertex();
  if (vertex != nullptr) {
    vertex->setId(_runningId++);
    graph()->addVertex(std::move(vertex));
  }
  return true;
}

bool World::addParameter(std::unique_ptr<Parameter> param) {
  if (graph() == nullptr) return false;
  param->setId(_paramId);
  graph()->addParameter(std::move(param));
  _paramId++;
  return true;
}

G2O_END_NAMESPACE
