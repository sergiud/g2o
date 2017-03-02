#ifndef G2O_SIMPLE_STAR_OPS_
#define G2O_SIMPLE_STAR_OPS_

#include "edge_creator.h"
#include "edge_labeler.h"
#include "star.h"
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/sparse_optimizer.h>
#include <map>
#include <string>

#include <g2o/hierarchical/g2o_hierarchical_api.h>

#include <g2o/prefix.hpp>

G2O_START_NAMESPACE


  G2O_HIERARCHICAL_API void constructEdgeStarMap(EdgeStarMap& esmap, StarSet& stars, bool low=true);

  size_t vertexEdgesInStar(HyperGraph::EdgeSet& eset, HyperGraph::Vertex* v, Star* s, EdgeStarMap& esmap);

  void starsInVertex(StarSet& stars, HyperGraph::Vertex* v, EdgeStarMap& esmap);

  void assignHierarchicalEdges(StarSet& stars, EdgeStarMap& esmap, EdgeLabeler* labeler, EdgeCreator* creator, SparseOptimizer* optimizer, int minNumEdges, int maxIterations);

  G2O_HIERARCHICAL_API void computeBorder(StarSet& stars, EdgeStarMap& hesmap);

  G2O_HIERARCHICAL_API void computeSimpleStars(StarSet& stars,
                          SparseOptimizer* optimizer,
                          EdgeLabeler* labeler,
                          EdgeCreator* creator,
                          OptimizableGraph::Vertex* gauge_,
                          std::string edgeTag, std::string vertexTag,
                          int level,
                          int step,
                          int backboneIterations=1,
                          int starIterations=30,
                          double rejectionThreshold=1e-5, bool debug=false);

G2O_END_NAMESPACE

#include <g2o/suffix.hpp>

#endif
