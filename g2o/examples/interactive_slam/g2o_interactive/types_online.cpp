// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

#include <iostream>

#include "types_slam2d_online.h"
#include "types_slam3d_online.h"

G2O_START_NAMESPACE
using namespace std;

G2O_ATTRIBUTE_CONSTRUCTOR(init_types_interactive_online) {
  Factory* factory = Factory::instance();
  // cerr << "Calling " << __FILE__ << " " << __PRETTY_FUNCTION__ << endl;

  factory->registerType("ONLINE_EDGE_SE2",
                        new HyperGraphElementCreator<OnlineEdgeSE2>);
  factory->registerType("ONLINE_VERTEX_SE2",
                        new HyperGraphElementCreator<OnlineVertexSE2>);

  factory->registerType("ONLINE_VERTEX_SE3:QUAT",
                        new HyperGraphElementCreator<OnlineVertexSE3>);
  factory->registerType("ONLINE_EDGE_SE3:QUAT",
                        new HyperGraphElementCreator<OnlineEdgeSE3>);
}

G2O_END_NAMESPACE
