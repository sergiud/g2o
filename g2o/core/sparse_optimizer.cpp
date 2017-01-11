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


#include <algorithm>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <cassert>
#include <algorithm>

#include <g2o/config.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/sparse_block_matrix.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/misc.h>
#include <g2o/stuff/timeutil.h>

G2O_START_NAMESPACE
  using namespace std;


  SparseOptimizer::SparseOptimizer() :
    _forceStopFlag(nullptr), _verbose(false), _algorithm(nullptr), _computeBatchStatistics(false)
  {
    _graphActions.resize(AT_NUM_ELEMENTS);
  }

  SparseOptimizer::~SparseOptimizer(){
    delete _algorithm;
    G2OBatchStatistics::setGlobalStats(nullptr);
  }

  void SparseOptimizer::computeActiveErrors()
  {
    // call the callbacks in case there is something registered
    HyperGraphActionSet& actions = _graphActions[AT_COMPUTEACTIVERROR];
    if (!actions.empty()) {
      for (auto action : actions)
        (*action)(this);
    }

#   ifdef G2O_OPENMP
#   pragma omp parallel for default (shared) if (_activeEdges.size() > 50)
#   endif
    for (auto e : _activeEdges) {
      e->computeError();
    }

#  ifndef NDEBUG
    for (int k = 0; k < static_cast<int>(_activeEdges.size()); ++k) {
      OptimizableGraph::Edge* e = _activeEdges[k];
      bool hasNan = arrayHasNaN(e->errorData(), e->dimension());
      if (hasNan) {
        cerr << "computeActiveErrors(): found NaN in error for edge " << e << endl;
      }
    }
#  endif

  }

  double SparseOptimizer::activeChi2( ) const
  {
    double chi = 0.0;
    for (auto e : _activeEdges) {
      chi += e->chi2();
    }
    return chi;
  }

  double SparseOptimizer::activeRobustChi2() const
  {
    Vector3D rho;
    double chi = 0.0;
    for (auto e : _activeEdges) {
      if (e->robustKernel() != nullptr) {
        e->robustKernel()->robustify(e->chi2(), rho);
        chi += rho[0];
      }
      else
        chi += e->chi2();
    }
    return chi;
  }

  OptimizableGraph::Vertex* SparseOptimizer::findGauge(){
    if (vertices().empty())
      return nullptr;

    int maxDim=0;
    for (auto & it : vertices()){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it.second);
      maxDim=std::max(maxDim,v->dimension());
    }

    OptimizableGraph::Vertex* rut=nullptr;
    for (auto & it : vertices()){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it.second);
      if (v->dimension()==maxDim){
        rut=v;
        break;
      }
    }
    return rut;
  }

  bool SparseOptimizer::gaugeFreedom()
  {
    if (vertices().empty())
      return false;

    int maxDim=0;
    for (auto & it : vertices()){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it.second);
      maxDim = std::max(maxDim,v->dimension());
    }

    for (auto & it : vertices()){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it.second);
      if (v->dimension() == maxDim) {
        // test for fixed vertex
        if (v->fixed()) {
          return false;
        }
        // test for full dimension prior
        for (auto eit : v->edges()) {
          OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(eit);
          if (e->vertices().size() == 1 && e->dimension() == maxDim)
            return false;
        }
      }
    }
    return true;
  }

  bool SparseOptimizer::buildIndexMapping(SparseOptimizer::VertexContainer& vlist){
    if (vlist.empty()){
      _ivMap.clear();
      return false;
    }

    _ivMap.resize(vlist.size());
    size_t i = 0;
    for (int k=0; k<2; k++)
      for (auto v : vlist){
      if (! v->fixed()){
        if (static_cast<int>(v->marginalized()) == k){
          v->setHessianIndex(i);
          _ivMap[i]=v;
          i++;
        }
      }
      else {
        v->setHessianIndex(-1);
      }
    }
    _ivMap.resize(i);
    return true;
  }

  void SparseOptimizer::clearIndexMapping(){
    for (auto & i : _ivMap){
      i->setHessianIndex(-1);
      i=nullptr;
    }
  }

  bool SparseOptimizer::initializeOptimization(int level){
    HyperGraph::VertexSet vset;
    for (auto & it : vertices())
      vset.insert(it.second);
    return initializeOptimization(vset,level);
  }

  bool SparseOptimizer::initializeOptimization(HyperGraph::VertexSet& vset, int level){
    if (edges().empty()) {
      cerr << __PRETTY_FUNCTION__ << ": Attempt to initialize an empty graph" << endl;
      return false;
    }
    preIteration(-1);
    bool workspaceAllocated = _jacobianWorkspace.allocate(); (void) workspaceAllocated;
    assert(workspaceAllocated && "Error while allocating memory for the Jacobians");
    clearIndexMapping();
    _activeVertices.clear();
    _activeVertices.reserve(vset.size());
    _activeEdges.clear();
    set<Edge*> auxEdgeSet; // temporary structure to avoid duplicates
    for (auto it=vset.begin(); it!=vset.end(); ++it){
      OptimizableGraph::Vertex* v= dynamic_cast<OptimizableGraph::Vertex*>( *it);
      const OptimizableGraph::EdgeSet& vEdges=v->edges();
      // count if there are edges in that level. If not remove from the pool
      int levelEdges=0;
      for (auto vEdge : vEdges){
        OptimizableGraph::Edge* e=reinterpret_cast<OptimizableGraph::Edge*>(vEdge);
        if (level < 0 || e->level() == level) {

          bool allVerticesOK = true;
          for (vector<HyperGraph::Vertex*>::const_iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
            if (vset.find(*vit) == vset.end()) {
              allVerticesOK = false;
              break;
            }
          }
          if (allVerticesOK && !e->allVerticesFixed()) {
            auxEdgeSet.insert(e);
            levelEdges++;
          }

        }
      }
      if (levelEdges != 0){
        _activeVertices.push_back(v);

        // test for NANs in the current estimate if we are debugging
#      ifndef NDEBUG
        int estimateDim = v->estimateDimension();
        if (estimateDim > 0) {
          VectorXD estimateData(estimateDim);
          if (v->getEstimateData(estimateData.data()) == true) {
            int k;
            bool hasNan = arrayHasNaN(estimateData.data(), estimateDim, &k);
            if (hasNan)
              cerr << __PRETTY_FUNCTION__ << ": Vertex " << v->id() << " contains a nan entry at index " << k << endl;
          }
        }
#      endif

      }
    }

    _activeEdges.reserve(auxEdgeSet.size());
    for (auto it : auxEdgeSet)
      _activeEdges.push_back(it);

    sortVectorContainers();
    bool indexMappingStatus = buildIndexMapping(_activeVertices);
    postIteration(-1);
    return indexMappingStatus;
  }

  bool SparseOptimizer::initializeOptimization(HyperGraph::EdgeSet& eset){
    preIteration(-1);
    bool workspaceAllocated = _jacobianWorkspace.allocate(); (void) workspaceAllocated;
    assert(workspaceAllocated && "Error while allocating memory for the Jacobians");
    clearIndexMapping();
    _activeVertices.clear();
    _activeEdges.clear();
    _activeEdges.reserve(eset.size());
    set<Vertex*> auxVertexSet; // temporary structure to avoid duplicates
    for (auto it : eset){
      OptimizableGraph::Edge* e=dynamic_cast<OptimizableGraph::Edge*>(it);
      if (e->numUndefinedVertices() != 0)
	continue;
      for (vector<HyperGraph::Vertex*>::const_iterator vit = e->vertices().begin(); vit != e->vertices().end(); ++vit) {
        auxVertexSet.insert(dynamic_cast<OptimizableGraph::Vertex*>(*vit));
      }
      _activeEdges.push_back(reinterpret_cast<OptimizableGraph::Edge*>(it));
    }

    _activeVertices.reserve(auxVertexSet.size());
    for (auto it : auxVertexSet)
      _activeVertices.push_back(it);

    sortVectorContainers();
    bool indexMappingStatus = buildIndexMapping(_activeVertices);
    postIteration(-1);
    return indexMappingStatus;
  }

  void SparseOptimizer::setToOrigin(){
    for (auto & it : vertices()) {
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it.second);
      v->setToOrigin();
    }
  }

  void SparseOptimizer::computeInitialGuess()
  {
    EstimatePropagator::PropagateCost costFunction(this);
    computeInitialGuess(costFunction);
  }

  void SparseOptimizer::computeInitialGuess(EstimatePropagatorCost& costFunction)
  {
    OptimizableGraph::VertexSet emptySet;
    std::set<Vertex*> backupVertices;
    HyperGraph::VertexSet fixedVertices; // these are the root nodes where to start the initialization
    for (auto e : _activeEdges) {
      for (size_t i = 0; i < e->vertices().size(); ++i) {
        OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(e->vertex(i));
	if (v == nullptr)
	  continue;
        if (v->fixed())
          fixedVertices.insert(v);
        else { // check for having a prior which is able to fully initialize a vertex
          for (auto vedgeIt = v->edges().begin(); vedgeIt != v->edges().end(); ++vedgeIt) {
            OptimizableGraph::Edge* vedge = dynamic_cast<OptimizableGraph::Edge*>(*vedgeIt);
            if (vedge->vertices().size() == 1 && vedge->initialEstimatePossible(emptySet, v) > 0.) {
              //cerr << "Initialize with prior for " << v->id() << endl;
              vedge->initialEstimate(emptySet, v);
              fixedVertices.insert(v);
            }
          }
        }
        if (v->hessianIndex() == -1) {
          auto foundIt = backupVertices.find(v);
          if (foundIt == backupVertices.end()) {
            v->push();
            backupVertices.insert(v);
          }
        }
      }
    }

    EstimatePropagator estimatePropagator(this);
    estimatePropagator.propagate(fixedVertices, costFunction);

    // restoring the vertices that should not be initialized
    for (auto v : backupVertices) {
      v->pop();
    }
    if (verbose()) {
      computeActiveErrors();
      cerr << "iteration= -1\t chi2= " << activeChi2()
          << "\t time= 0.0"
          << "\t cumTime= 0.0"
          << "\t (using initial guess from " << costFunction.name() << ")" << endl;
    }
  }

  int SparseOptimizer::optimize(int iterations, bool online)
  {
    if (_ivMap.empty()) {
      cerr << __PRETTY_FUNCTION__ << ": 0 vertices to optimize, maybe forgot to call initializeOptimization()" << endl;
      return -1;
    }

    int cjIterations=0;
    double cumTime=0;
    bool ok=true;

    ok = _algorithm->init(online);
    if (! ok) {
      cerr << __PRETTY_FUNCTION__ << " Error while initializing" << endl;
      return -1;
    }

    _batchStatistics.clear();
    if (_computeBatchStatistics)
      _batchStatistics.resize(iterations);

    OptimizationAlgorithm::SolverResult result = OptimizationAlgorithm::OK;
    for (int i=0; i<iterations && ! terminate() && ok; i++){
      preIteration(i);

      if (_computeBatchStatistics) {
        G2OBatchStatistics& cstat = _batchStatistics[i];
        G2OBatchStatistics::setGlobalStats(&cstat);
        cstat.iteration = i;
        cstat.numEdges =  _activeEdges.size();
        cstat.numVertices = _activeVertices.size();
      }

      double ts = get_monotonic_time();
      result = _algorithm->solve(i, online);
      ok = ( result == OptimizationAlgorithm::OK );

      bool errorComputed = false;
      if (_computeBatchStatistics) {
        computeActiveErrors();
        errorComputed = true;
        _batchStatistics[i].chi2 = activeRobustChi2();
        _batchStatistics[i].timeIteration = get_monotonic_time()-ts;
      }

      if (verbose()){
        double dts = get_monotonic_time()-ts;
        cumTime += dts;
        if (! errorComputed)
          computeActiveErrors();
        cerr << "iteration= " << i
          << "\t chi2= " << FIXED(activeRobustChi2())
          << "\t time= " << dts
          << "\t cumTime= " << cumTime
          << "\t edges= " << _activeEdges.size();
        _algorithm->printVerbose(cerr);
        cerr << endl;
      }
      ++cjIterations;
      postIteration(i);
    }
    if (result == OptimizationAlgorithm::Fail) {
      return 0;
    }
    return cjIterations;
  }

  void SparseOptimizer::update(const double* update)
  {
    // update the graph by calling oplus on the vertices
    for (auto v : _ivMap) {

#ifndef NDEBUG
      bool hasNan = arrayHasNaN(update, v->dimension());
      if (hasNan)
        cerr << __PRETTY_FUNCTION__ << ": Update contains a nan for vertex " << v->id() << endl;
#endif
      v->oplus(update);
      update += v->dimension();
    }
  }

  void SparseOptimizer::setComputeBatchStatistics(bool computeBatchStatistics)
  {
    if ((_computeBatchStatistics == true) && (computeBatchStatistics == false)) {
      G2OBatchStatistics::setGlobalStats(nullptr);
      _batchStatistics.clear();
    }
    _computeBatchStatistics = computeBatchStatistics;
  }

  bool SparseOptimizer::updateInitialization(HyperGraph::VertexSet& vset, HyperGraph::EdgeSet& eset)
  {
    std::vector<HyperGraph::Vertex*> newVertices;
    newVertices.reserve(vset.size());
    _activeVertices.reserve(_activeVertices.size() + vset.size());
    _activeEdges.reserve(_activeEdges.size() + eset.size());
    for (auto it : eset) {
      OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(it);
      if (!e->allVerticesFixed()) _activeEdges.push_back(e);
    }

    // update the index mapping
    size_t next = _ivMap.size();
    for (auto it : vset) {
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it);
      if (! v->fixed()){
        if (! v->marginalized()){
          v->setHessianIndex(next);
          _ivMap.push_back(v);
          newVertices.push_back(v);
          _activeVertices.push_back(v);
          next++;
        }
        else // not supported right now
          abort();
      }
      else {
        v->setHessianIndex(-1);
      }
    }

    //if (newVertices.size() != vset.size())
    //cerr << __PRETTY_FUNCTION__ << ": something went wrong " << PVAR(vset.size()) << " " << PVAR(newVertices.size()) << endl;
    return _algorithm->updateStructure(newVertices, eset);
  }

  void SparseOptimizer::sortVectorContainers()
  {
    // sort vector structures to get deterministic ordering based on IDs
    sort(_activeVertices.begin(), _activeVertices.end(), VertexIDCompare());
    sort(_activeEdges.begin(), _activeEdges.end(), EdgeIDCompare());
  }

  void SparseOptimizer::clear() {
    _ivMap.clear();
    _activeVertices.clear();
    _activeEdges.clear();
    OptimizableGraph::clear();
  }

  SparseOptimizer::VertexContainer::const_iterator SparseOptimizer::findActiveVertex(const OptimizableGraph::Vertex* v) const
  {
    auto lower = lower_bound(_activeVertices.begin(), _activeVertices.end(), v, VertexIDCompare());
    if (lower == _activeVertices.end())
      return _activeVertices.end();
    if ((*lower) == v)
      return lower;
    return _activeVertices.end();
  }

  SparseOptimizer::EdgeContainer::const_iterator SparseOptimizer::findActiveEdge(const OptimizableGraph::Edge* e) const
  {
    auto lower = lower_bound(_activeEdges.begin(), _activeEdges.end(), e, EdgeIDCompare());
    if (lower == _activeEdges.end())
      return _activeEdges.end();
    if ((*lower) == e)
      return lower;
    return _activeEdges.end();
  }

  void SparseOptimizer::push(SparseOptimizer::VertexContainer& vlist)
  {
    for (auto & it : vlist)
      it->push();
  }

  void SparseOptimizer::pop(SparseOptimizer::VertexContainer& vlist)
  {
    for (auto & it : vlist)
      it->pop();
  }

  void SparseOptimizer::push(HyperGraph::VertexSet& vlist)
  {
    for (auto it : vlist) {
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
      if (v != nullptr)
	v->push();
      else
	cerr << __FUNCTION__ << ": FATAL PUSH SET" << endl;
    }
  }

  void SparseOptimizer::pop(HyperGraph::VertexSet& vlist)
  {
    for (auto it : vlist){
      OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*> (it);
      if (v != nullptr)
	v->pop();
      else
	cerr << __FUNCTION__ << ": FATAL POP SET" << endl;
    }
  }

  void SparseOptimizer::discardTop(SparseOptimizer::VertexContainer& vlist)
  {
    for (auto & it : vlist)
      it->discardTop();
  }

  void SparseOptimizer::setVerbose(bool verbose)
  {
    _verbose = verbose;
  }

  void SparseOptimizer::setAlgorithm(OptimizationAlgorithm* algorithm)
  {
    if (_algorithm != nullptr) // reset the optimizer for the formerly used solver
      _algorithm->setOptimizer(nullptr);
    _algorithm = algorithm;
    if (_algorithm != nullptr)
      _algorithm->setOptimizer(this);
  }

  bool SparseOptimizer::computeMarginals(SparseBlockMatrix<MatrixXD>& spinv, const std::vector<std::pair<int, int> >& blockIndices){
    return _algorithm->computeMarginals(spinv, blockIndices);
  }

  void SparseOptimizer::setForceStopFlag(bool* flag)
  {
    _forceStopFlag=flag;
  }

  bool SparseOptimizer::removeVertex(HyperGraph::Vertex* v, bool detach)
  {
    OptimizableGraph::Vertex* vv = dynamic_cast<OptimizableGraph::Vertex*>(v);
    if (vv->hessianIndex() >= 0) {
      clearIndexMapping();
      _ivMap.clear();
    }
    return HyperGraph::removeVertex(v, detach);
  }

  bool SparseOptimizer::addComputeErrorAction(HyperGraphAction* action)
  {
    std::pair<HyperGraphActionSet::iterator, bool> insertResult = _graphActions[AT_COMPUTEACTIVERROR].insert(action);
    return insertResult.second;
  }

  bool SparseOptimizer::removeComputeErrorAction(HyperGraphAction* action)
  {
    return _graphActions[AT_COMPUTEACTIVERROR].erase(action) > 0;
  }

  void SparseOptimizer::push()
  {
    push(_activeVertices);
  }

  void SparseOptimizer::pop()
  {
    pop(_activeVertices);
  }

  void SparseOptimizer::discardTop()
  {
    discardTop(_activeVertices);
  }

G2O_END_NAMESPACE
