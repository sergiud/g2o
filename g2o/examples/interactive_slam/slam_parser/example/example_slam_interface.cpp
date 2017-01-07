// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "example_slam_interface.h"

#include <iostream>
using namespace std;

ExampleSlamInterface::ExampleSlamInterface()
{
}

bool ExampleSlamInterface::addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values)
{
  cerr << "ADDING NODE " << tag << " id=" << id << " dim=" << dimension;
  if (!values.empty() != 0u) {
    cerr << "\tpose=";
    for (double value : values)
      cerr << " " << value; 
  }
  cerr << endl;

  // store the values
  if (values.empty())
    _vertices[id] = make_pair(tag, std::vector<double>(dimension));
  else
    _vertices[id] = make_pair(tag, values);

  return true;
}

bool ExampleSlamInterface::addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information)
{
  cerr << "ADDING EDGE " << tag << " id=" << id << " dim=" << dimension
    << " (" << v1 << " <-> " << v2 << ")" << " measurement=";
  for (double i : measurement)
    cerr << " " << i; 
  cerr << " information=";
  for (double i : information)
    cerr << " " << i; 
  cerr << endl;
  return true;
}

bool ExampleSlamInterface::fixNode(const std::vector<int>& nodes)
{
  cerr << "FIXING NODE";
  for (int node : nodes)
    cerr << " " << node; 
  cerr << endl;
  return true;
}

bool ExampleSlamInterface::queryState(const std::vector<int>& nodes)
{
  cerr << "QUERY STATE";
  for (int node : nodes)
    cerr << " " << node; 
  cerr << endl;

  // actually output the values to the evaluator
  // If a SLAM algorithm is running we would need to copy its estimate
  if (nodes.empty()) {
    // print all nodes
    for (std::map<int, std::pair<std::string, std::vector<double> > >::const_iterator it = _vertices.begin();
        it != _vertices.end(); ++it) {
      cout << it->second.first << " " << it->first;
      const vector<double>& values = it->second.second;
      for (double value : values)
        cout << " " << value;
      cout << endl;
    }
  } else {
    for (int node : nodes) {
      std::map<int, std::pair<std::string, std::vector<double> > >::const_iterator it = _vertices.find(node);
      if (it != _vertices.end()) {
        cout << it->second.first << " " << it->first;
        const vector<double>& values = it->second.second;
        for (double value : values)
          cout << " " << value;
        cout << endl;
      }
    }
  }

  return true;
}

bool ExampleSlamInterface::solveState()
{
  cerr << "SOLVE STATE" << endl;
  return true;
}
