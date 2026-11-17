// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_AIS_GRAPH_TOOLS_HH
#define G2O_AIS_GRAPH_TOOLS_HH

#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/hyper_graph.h>

G2O_START_NAMESPACE

void findConnectedEdgesWithCostLimit(HyperGraph::EdgeSet& selected,
                                     HyperGraph::EdgeSet& border,
                                     HyperGraph::Edge* start,
                                     HyperDijkstra::CostFunction* cost,
                                     double maxEdgeCost,
                                     double comparisonConditioner = 1e-6);
}
;

#endif
