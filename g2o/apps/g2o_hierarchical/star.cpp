#include "star.h"
#include <g2o/core/optimization_algorithm_with_hessian.h>

G2O_START_NAMESPACE
  using namespace std;

  Star::Star(int level, SparseOptimizer* optimizer): _level(level), _optimizer(optimizer) {}

  bool Star::labelStarEdges(int iterations, EdgeLabeler* labeler){
    // mark all vertices in the lowLevelEdges as floating
    bool ok=true;
    std::set<OptimizableGraph::Vertex*> vset;
    for (auto e : _lowLevelEdges){
      for (auto & i : e->vertices()){
        OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(i);
        v->setFixed(false);
        vset.insert(v);
      }
    }
    for (auto v : vset){
      v->push();
    }

    // fix all vertices in the gauge
    //cerr << "fixing gauge: ";
    for (auto it : _gauge){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it);
      //cerr << v->id() << " ";
      v->setFixed(true);
    }
    //cerr << endl;
    if (iterations>0){
      _optimizer->initializeOptimization(_lowLevelEdges);
      _optimizer->computeInitialGuess();
      int result=_optimizer->optimize(iterations);
      if (result<1){
        cerr << "Vertices num: " << _optimizer->activeVertices().size() << "ids: ";
        for (auto i : _optimizer->indexMapping()){
          cerr << i->id() << " " ;
        }
        cerr << endl;
        cerr << "!!! optimization failure" << endl;
        cerr << "star size=" << _lowLevelEdges.size() << endl;
        cerr << "gauge: ";
        for (auto it : _gauge){
          OptimizableGraph::Vertex* v = dynamic_cast<OptimizableGraph::Vertex*>(it);
          cerr << "[" << v->id() << " " << v->hessianIndex() << "] ";
        }
        cerr << endl;
        ok=false;
      }
    }  else {
      optimizer()->initializeOptimization(_lowLevelEdges);
      // cerr << "guess" << endl;
      //optimizer()->computeInitialGuess();
      // cerr << "solver init" << endl;
      optimizer()->solver()->init();
      // cerr << "structure" << endl;
      OptimizationAlgorithmWithHessian* solverWithHessian = dynamic_cast<OptimizationAlgorithmWithHessian*> (optimizer()->solver());
      if (!solverWithHessian->buildLinearStructure())
        cerr << "FATAL: failure while building linear structure" << endl;
      // cerr << "errors" << endl;
      optimizer()->computeActiveErrors();
      // cerr << "system" << endl;
      solverWithHessian->updateLinearSystem();
    }

    std::set<OptimizableGraph::Edge*> star;
    for(auto _starEdge : _starEdges){
      star.insert(dynamic_cast<OptimizableGraph::Edge*>(_starEdge));
    }
    if (ok) {
      int result = labeler->labelEdges(star);
      if (result < 0)
        ok=false;
    }
    // release all vertices in the gauge
    for (auto v : vset){
      v->pop();
    }
    for (auto it : _gauge){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(it);
      v->setFixed(false);
    }

    return ok;
  }

G2O_END_NAMESPACE
