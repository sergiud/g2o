#include "simple_star_ops.h"
#include "backbone_tree_action.h"
#include "edge_types_cost_function.h"
#include <g2o/core/optimization_algorithm_with_hessian.h>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <iostream>


namespace g2o{

  using namespace std;
  using namespace Eigen;

  double activeVertexChi(const OptimizableGraph::Vertex* v){
    const SparseOptimizer* s = dynamic_cast<const SparseOptimizer*>(v->graph());
    const OptimizableGraph::EdgeContainer& av = s->activeEdges();
    double chi = 0;
    int ne =0;
    for (auto it : v->edges()){
      OptimizableGraph::Edge* e = dynamic_cast <OptimizableGraph::Edge*> (it);
      if (e == nullptr)
	continue;
      if (s->findActiveEdge(e)!=av.end()) {
	chi +=e->chi2();
	ne++;
      }
    }
    if (ne == 0)
      return -1;
    return chi/ne;
  }

void constructEdgeStarMap(EdgeStarMap& esmap, StarSet& stars, bool low){
  esmap.clear();
  for (auto s : stars){
    if (low) {
      for (auto it = s->lowLevelEdges().begin();
          it!=s->lowLevelEdges().end(); it++){
        HyperGraph::Edge* e=*it;
        esmap.insert(make_pair(e,s));
      }
    } else {
      for (auto it = s->starEdges().begin();
          it!=s->starEdges().end(); it++){
        HyperGraph::Edge* e=*it;
        esmap.insert(make_pair(e,s));
      }
    }
  }
}

size_t vertexEdgesInStar(HyperGraph::EdgeSet& eset, HyperGraph::Vertex* v, Star* s, EdgeStarMap& esmap){
  eset.clear();
  for (auto e : v->edges()){
    auto eit=esmap.find(e);
    if (eit!=esmap.end() && eit->second == s)
      eset.insert(e);
  }
  return eset.size();
}

void starsInVertex(StarSet& stars, HyperGraph::Vertex* v, EdgeStarMap& esmap){
  for (auto e : v->edges()){
    auto eit=esmap.find(e);
    if (eit!=esmap.end())
      stars.insert(eit->second);
  }
}

void starsInEdge(StarSet& stars, HyperGraph::Edge* e, EdgeStarMap& esmap, HyperGraph::VertexSet& gauge){
  for (auto & i : e->vertices()){
    OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(i);
    if (gauge.find(v)==gauge.end())
      starsInVertex(stars, v, esmap);
  }
}

void assignHierarchicalEdges(StarSet& stars, EdgeStarMap& esmap, EdgeLabeler* labeler, EdgeCreator* creator, SparseOptimizer* optimizer, int minNumEdges, int maxIterations){
  // now construct the hierarchical edges for all the stars
  int starNum=0;
  for (auto s : stars){
    cerr << "STAR# " << starNum << endl;
    std::vector<OptimizableGraph::Vertex*> vertices(2);
    vertices[0]= dynamic_cast<OptimizableGraph::Vertex*>( *s->_gauge.begin());
    cerr << "eIs"  << endl;
    HyperGraph::VertexSet vNew =s->lowLevelVertices();
    for (auto vit=s->_lowLevelVertices.begin(); vit!=s->_lowLevelVertices.end(); vit++){
      OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(*vit);
      vertices[1]=v;
      if (v==vertices[0])
        continue;
      HyperGraph::EdgeSet eInSt;
      int numEdges = vertexEdgesInStar(eInSt, v, s, esmap);
      if (Factory::instance()->tag(v)==Factory::instance()->tag(vertices[0]) || numEdges>minNumEdges) {
        OptimizableGraph::Edge* e=creator->createEdge(vertices);
        //cerr << "creating edge" << e << endl;
        if (e != nullptr) {
          e->setLevel(1);
          optimizer->addEdge(e);
          s->_starEdges.insert(e);
        } else {
          cerr << "THERE" << endl;
          cerr << "FATAL, cannot create edge" << endl;
        }
      } else {
        vNew.erase(v);
        // cerr << numEdges << " ";
        // cerr << "r " <<  v-> id() << endl;
        // remove from the star all edges that are not sufficiently connected
        for (auto e : eInSt){
          s->lowLevelEdges().erase(e);
        }
      }
    }
    s->lowLevelVertices()=vNew;
    //cerr << endl;
    cerr <<  "gauge: " << (*s->_gauge.begin())->id()
      << " edges:" << s->_lowLevelEdges.size()
      << " hedges" << s->_starEdges.size() << endl;

    const bool debug = false;
    if (debug){
      char starLowName[100];
      sprintf(starLowName, "star-%04d-low.g2o", starNum);
      ofstream starLowStream(starLowName);
      optimizer->saveSubset(starLowStream, s->_lowLevelEdges);
    }
    bool labelOk=s->labelStarEdges(maxIterations, labeler);
    if (labelOk) {
      if (debug) {
        char starHighName[100];
        sprintf(starHighName, "star-%04d-high.g2o", starNum);
        ofstream starHighStream(starHighName);
        optimizer->saveSubset(starHighStream, s->_starEdges);
      }
    } else {
      cerr << "FAILURE" << endl;
    }
    starNum++;
  }
}

void computeBorder(StarSet& stars, EdgeStarMap& hesmap){
  cerr << "computing edges on the border" << endl;
  for (auto s : stars){
    for (auto iit=s->_starEdges.begin(); iit!=s->_starEdges.end(); iit++){
      OptimizableGraph::Edge* e= dynamic_cast<OptimizableGraph::Edge*>( *iit);
      StarSet sset;
      starsInEdge(sset, e, hesmap, s->gauge());
      //cerr << "e: " << e << " l:" << e->level() << " sset.size()=" << sset.size() << endl;
      if (sset.size()>1){
        s->starFrontierEdges().insert(e);
      }
    }
  }
}


  void computeSimpleStars(StarSet& stars,
			  SparseOptimizer* optimizer,
			  EdgeLabeler* labeler,
			  EdgeCreator* creator,
			  OptimizableGraph::Vertex* gauge_,
			  std::string edgeTag,
			  std::string vertexTag,
			  int level,
			  int step,
			  int backboneIterations,
			  int starIterations,
			  double rejectionThreshold,
			  bool debug){

    cerr << "preforming the tree actions" << endl;
    HyperDijkstra d(optimizer);
    // compute a spanning tree based on the types of edges and vertices in the pool
    EdgeTypesCostFunction f(edgeTag, vertexTag, level);
    d.shortestPaths(gauge_,
        &f,
        std::numeric_limits< double >::max(),
        1e-6,
        false,
        std::numeric_limits< double >::max()/2);

    HyperDijkstra::computeTree(d.adjacencyMap());
    // constructs the stars on the backbone

    BackBoneTreeAction bact(optimizer, vertexTag, level, step);
    bact.init();

    cerr << "free edges size " << bact.freeEdges().size() << endl;

    // perform breadth-first visit of the visit tree and create the stars on the backbone
    d.visitAdjacencyMap(d.adjacencyMap(),&bact,true);
    stars.clear();

    for (auto & it : bact.vertexStarMultiMap()){
      stars.insert(it.second);
    }
    cerr << "stars.size: " << stars.size() << endl;
    cerr << "size: " << bact.vertexStarMultiMap().size() << endl;


    //  for each star

    //    for all vertices in the backbone, select all edges leading/leaving from that vertex
    //    that are contained in freeEdges.

    //      mark the corresponding "open" vertices and add them to a multimap (vertex->star)

    //    select a gauge in the backbone

    //    push all vertices on the backbone

    //    compute an initial guess on the backbone

    //    one round of optimization backbone

    //    lock all vertices in the backbone

    //    push all "open" vertices

    //    for each open vertex,
    //      compute an initial guess given the backbone
    //      do some rounds of solveDirect
    //      if (fail)
    //        - remove the vertex and the edges in that vertex from the star
    //   - make the structures consistent

    //    pop all "open" vertices
    //    pop all "vertices" in the backbone
    //    unfix the vertices in the backbone

    int starNum=0;
    for (auto s : stars){
      HyperGraph::VertexSet backboneVertices = s->_lowLevelVertices;
      HyperGraph::EdgeSet backboneEdges = s->_lowLevelEdges;
      if (backboneEdges.empty())
	continue;


      // cerr << "optimizing backbone" << endl;
      // one of these  should be the gauge, to be simple we select the fisrt one in the backbone
      OptimizableGraph::VertexSet gauge;
      gauge.insert(*backboneVertices.begin());
      s->gauge()=gauge;
      s->optimizer()->push(backboneVertices);
      s->optimizer()->setFixed(gauge,true);
      s->optimizer()->initializeOptimization(backboneEdges);
      s->optimizer()->computeInitialGuess();
      s->optimizer()->optimize(backboneIterations);
      s->optimizer()->setFixed(backboneVertices, true);

      // cerr << "assignind edges.vertices not in bbone" << endl;
      HyperGraph::EdgeSet otherEdges;
      HyperGraph::VertexSet otherVertices;
      std::multimap<HyperGraph::Vertex*, HyperGraph::Edge*> vemap;
      for (auto bit=backboneVertices.begin(); bit!=backboneVertices.end(); bit++){
	HyperGraph::Vertex* v=*bit;
	for (auto eit : v->edges()){
	  OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>( eit);
	  auto feit=bact.freeEdges().find(e);
	  if (feit!=bact.freeEdges().end()){ // edge is admissible
	    otherEdges.insert(e);
	    bact.freeEdges().erase(feit);
	    for (size_t i=0; i<e->vertices().size(); i++){
	      OptimizableGraph::Vertex* ve= dynamic_cast<OptimizableGraph::Vertex*>(e->vertices()[i]);
	      if (backboneVertices.find(ve)==backboneVertices.end()){
		otherVertices.insert(ve);
		vemap.insert(make_pair(ve,e));
	      }
	    }
	  }
	}
      }

      // RAINER TODO maybe need a better solution than dynamic casting here??
      OptimizationAlgorithmWithHessian* solverWithHessian = dynamic_cast<OptimizationAlgorithmWithHessian*>(s->optimizer()->solver());
      if (solverWithHessian != nullptr) {
        s->optimizer()->push(otherVertices);
        // cerr << "optimizing vertices out of bbone" << endl;
        // cerr << "push" << endl;
        // cerr << "init" << endl;
        s->optimizer()->initializeOptimization(otherEdges);
        // cerr << "guess" << endl;
        s->optimizer()->computeInitialGuess();
        // cerr << "solver init" << endl;
        s->optimizer()->solver()->init();
        // cerr << "structure" << endl;
        if (!solverWithHessian->buildLinearStructure())
          cerr << "FATAL: failure while building linear structure" << endl;
        // cerr << "errors" << endl;
        s->optimizer()->computeActiveErrors();
        // cerr << "system" << endl;
        solverWithHessian->updateLinearSystem();
        // cerr << "directSolove" << endl;
      } else {
        cerr << "FATAL: hierarchical thing cannot be used with a solver that does not support the system structure construction" << endl;
      }


      // // then optimize the vertices one at a time to check if a solution is good
      for (auto otherVertice : otherVertices){
        OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(otherVertice);
        v->solveDirect();
        // cerr << " " << d;
        // if  a solution is found, add a vertex and all the edges in
        //othervertices that are pointing to that edge to the star
        s->_lowLevelVertices.insert(v);
        for (auto eit : v->edges()){
          OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>( eit);
          if (otherEdges.find(e)!=otherEdges.end())
            s->_lowLevelEdges.insert(e);
        }
      }
      //cerr <<  endl;

      // relax the backbone and optimize it all
      // cerr << "relax bbone" << endl;
      s->optimizer()->setFixed(backboneVertices, false);
      //cerr << "fox gauge bbone" << endl;
      s->optimizer()->setFixed(s->gauge(),true);

      //cerr << "opt init" << endl;
      s->optimizer()->initializeOptimization(s->_lowLevelEdges);
      optimizer->computeActiveErrors();
      double initialChi = optimizer->activeChi2();
      int starOptResult = s->optimizer()->optimize(starIterations);
      //cerr << starOptResult << "(" << starIterations << ")  " << endl;
      double finalchi=-1.;

      cerr <<  "computing star: " << starNum << endl;

      int vKept=0, vDropped=0;
      if ((starIterations == 0) || starOptResult > 0  ){
	optimizer->computeActiveErrors();
	finalchi = optimizer->activeChi2();

#if 1

        s->optimizer()->computeActiveErrors();
        // cerr << "system" << endl;
        if (solverWithHessian != nullptr)
          solverWithHessian->updateLinearSystem();
        HyperGraph::EdgeSet prunedStarEdges = backboneEdges;
        HyperGraph::VertexSet prunedStarVertices = backboneVertices;
        for (auto otherVertice : otherVertices){

	  //discard the vertices whose error is too big


          OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(otherVertice);
          MatrixXd h(v->dimension(), v->dimension());
          for (int i=0; i<v->dimension(); i++){
            for (int j=0; j<v->dimension(); j++)
              h(i,j)=v->hessian(i,j);
          }
          EigenSolver<Eigen::MatrixXd> esolver;
          esolver.compute(h);
          VectorXcd ev= esolver.eigenvalues();
          double emin = std::numeric_limits<double>::max();
          double emax = -std::numeric_limits<double>::max();
          for (int i=0; i<ev.size(); i++){
            emin = ev(i).real()>emin ? emin : ev(i).real();
            emax = ev(i).real()<emax ? emax : ev(i).real();
          }

          double d=emin/emax;


          // cerr << " " << d;
          if (d>rejectionThreshold){
	  // if  a solution is found, add a vertex and all the edges in
	  //othervertices that are pointing to that edge to the star
            prunedStarVertices.insert(v);
            for (auto eit : v->edges()){
              OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>( eit);
              if (otherEdges.find(e)!=otherEdges.end())
                prunedStarEdges.insert(e);
            }
            //cerr << "K( " << v->id() << "," << d << ")" ;
            vKept ++;
          } else {
            vDropped++;
            //cerr << "R( " << v->id() << "," << d << ")" ;
          }
        }
        s->_lowLevelEdges=prunedStarEdges;
        s->_lowLevelVertices=prunedStarVertices;

#endif
	//cerr << "addHedges" << endl;
	//now add to the star the hierarchical edges
	std::vector<OptimizableGraph::Vertex*> vertices(2);
	vertices[0]= dynamic_cast<OptimizableGraph::Vertex*>( *s->_gauge.begin());

	for (auto vit=s->_lowLevelVertices.begin(); vit!=s->_lowLevelVertices.end(); vit++){
	  OptimizableGraph::Vertex* v=dynamic_cast<OptimizableGraph::Vertex*>(*vit);
	  vertices[1]=v;
	  if (v==vertices[0])
	    continue;
	  OptimizableGraph::Edge* e=creator->createEdge(vertices);
	  //rr << "creating edge" << e <<  Factory::instance()->tag(vertices[0]) << "->" <<  Factory::instance()->tag(v) <endl;
	  if (e != nullptr) {
	    e->setLevel(level+1);
	    optimizer->addEdge(e);
	    s->_starEdges.insert(e);
	  } else {
            cerr << "HERE" << endl;
	    cerr << "FATAL, cannot create edge" << endl;
	  }
	}
      }

      cerr << " gauge: " << (*s->_gauge.begin())->id()
           << " kept: " << vKept
           << " dropped: " << vDropped
	   << " edges:" << s->_lowLevelEdges.size()
	   << " hedges" << s->_starEdges.size()
	   << " initial chi " << initialChi
	   << " final chi " << finalchi << endl;

      if (debug) {
	char starLowName[100];
	sprintf(starLowName, "star-%04d-low.g2o", starNum);
	ofstream starLowStream(starLowName);
	optimizer->saveSubset(starLowStream, s->_lowLevelEdges);
      }
      bool labelOk=false;
      if ((starIterations == 0) || starOptResult > 0)
        labelOk = s->labelStarEdges(0, labeler);
      if (labelOk) {
        if (debug) {
          char starHighName[100];
          sprintf(starHighName, "star-%04d-high.g2o", starNum);
          ofstream starHighStream(starHighName);
          optimizer->saveSubset(starHighStream, s->_starEdges);
        }
      } else {

        cerr << "FAILURE: " << starOptResult << endl;
      }
      starNum++;

      //label each hierarchical edge
      s->optimizer()->pop(otherVertices);
      s->optimizer()->pop(backboneVertices);
      s->optimizer()->setFixed(s->gauge(),false);
    }


    StarSet stars2;
    // now erase the stars that have 0 edges. They r useless
    for (auto s : stars){
      if (s->lowLevelEdges().empty()) {
        delete s;
      } else
        stars2.insert(s);
    }
    stars=stars2;
  }


} // namespace g2o
