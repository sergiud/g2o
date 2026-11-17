// SPDX-FileCopyrightText: 2011 G. Grisetti, R. Kuemmerle, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "simulator.h"

#include <iostream>

G2O_START_NAMESPACE
using namespace std;

// BaseWorldObject
BaseWorldObject::~BaseWorldObject() = default;
OptimizableGraph* BaseWorldObject::graph() {
  if (_world != nullptr) return _world->graph();
  return nullptr;
}

void BaseWorldObject::setVertex(OptimizableGraph::Vertex* vertex_) {
  _vertex = vertex_;
}

// BaseRobot
OptimizableGraph* BaseRobot::graph() {
  if (_world != nullptr) return _world->graph();
  return nullptr;
}

bool BaseRobot::addSensor(BaseSensor* sensor) {
  assert(graph());
  std::pair<std::set<BaseSensor*>::iterator, bool> result =
      _sensors.insert(sensor);
  if (result.second) {
    sensor->setRobot(this);
    sensor->addParameters();
  }
  return result.second;
}

void BaseRobot::sense() {
  for (auto s : _sensors) {
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

bool World::addWorldObject(BaseWorldObject* object) {
  std::pair<std::set<BaseWorldObject*>::iterator, bool> result =
      _objects.insert(object);
  if (result.second) {
    object->setWorld(this);
  }
  if ((graph() != nullptr) && (object->vertex() != nullptr)) {
    object->vertex()->setId(_runningId++);
    graph()->addVertex(object->vertex());
  }
  return result.second;
}

bool World::addParameter(Parameter* param) {
  if (graph() == nullptr) return false;
  param->setId(_paramId);
  graph()->addParameter(param);
  _paramId++;
  return true;
}

G2O_END_NAMESPACE
