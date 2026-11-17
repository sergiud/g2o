// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "example_slam_interface.h"

#include <g2o/stuff/logger.h>

#include <iostream>
using namespace std;

ExampleSlamInterface::ExampleSlamInterface() {}

bool ExampleSlamInterface::addNode(const std::string& tag, int id,
                                   int dimension,
                                   const std::vector<double>& values) {
  std::string pose;
  if (!values.empty()) {
    pose = "\tpose=";
    for (double value : values) {
      std::format_to(std::back_inserter(pose), " {}", value);
    }
  }
  g2o::ngx::Log("ADDING NODE {} id={} dim={}{}", tag, id, dimension, pose);

  // store the values
  if (values.empty())
    _vertices[id] = make_pair(tag, std::vector<double>(dimension));
  else
    _vertices[id] = make_pair(tag, values);

  return true;
}

bool ExampleSlamInterface::addEdge(const std::string& tag, int id,
                                   int dimension, int v1, int v2,
                                   const std::vector<double>& measurement,
                                   const std::vector<double>& information) {
  std::string values;
  for (double i : measurement) {
    std::format_to(std::back_inserter(values), " {}", i);
  }
  values += " information=";
  for (double i : information) {
    std::format_to(std::back_inserter(values), " {}", i);
  }
  g2o::ngx::Log("ADDING EDGE {} id={} dim={} ({} <-> {}) measurement={}", tag,
                id, dimension, v1, v2, values);
  return true;
}

bool ExampleSlamInterface::fixNode(const std::vector<int>& nodes) {
  std::string ids;
  for (int node : nodes) {
    std::format_to(std::back_inserter(ids), " {}", node);
  }
  g2o::ngx::Log("FIXING NODE{}", ids);
  return true;
}

bool ExampleSlamInterface::queryState(const std::vector<int>& nodes) {
  std::string ids;
  for (int node : nodes) {
    std::format_to(std::back_inserter(ids), " {}", node);
  }
  g2o::ngx::Log("QUERY STATE{}", ids);

  // actually output the values to the evaluator
  // If a SLAM algorithm is running we would need to copy its estimate
  if (nodes.empty()) {
    // print all nodes
    for (std::map<int,
                  std::pair<std::string, std::vector<double> > >::const_iterator
             it = _vertices.begin();
         it != _vertices.end(); ++it) {
      cout << it->second.first << " " << it->first;
      const vector<double>& values = it->second.second;
      for (double value : values) cout << " " << value;
      cout << endl;
    }
  } else {
    for (int node : nodes) {
      std::map<int,
               std::pair<std::string, std::vector<double> > >::const_iterator
          it = _vertices.find(node);
      if (it != _vertices.end()) {
        cout << it->second.first << " " << it->first;
        const vector<double>& values = it->second.second;
        for (double value : values) cout << " " << value;
        cout << endl;
      }
    }
  }

  return true;
}

bool ExampleSlamInterface::solveState() {
  g2o::ngx::Log("SOLVE STATE");
  return true;
}
