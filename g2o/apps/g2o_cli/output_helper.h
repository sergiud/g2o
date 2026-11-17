// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_OUTPUT_HELPER_H
#define G2O_OUTPUT_HELPER_H

#include <g2o/cli/g2o_cli_api.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/prefix.h>

#include <iosfwd>
#include <string>

G2O_START_NAMESPACE

struct OptimizableGraph;

/**
 * save the state of the optimizer into files for visualizing using Gnuplot
 */
G2O_CLI_API bool saveGnuplot(const std::string& gnudump,
                             const OptimizableGraph& optimizer);
G2O_CLI_API bool saveGnuplot(const std::string& gnudump,
                             const HyperGraph::VertexSet& vertices,
                             const HyperGraph::EdgeSet& edges);

/**
 * dump the edges to a stream, e.g., cout and redirect to gnuplot
 */
G2O_CLI_API bool dumpEdges(std::ostream& os, const OptimizableGraph& optimizer);

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
