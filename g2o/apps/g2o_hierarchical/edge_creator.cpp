#include <g2o/core/factory.h>
#include "edge_creator.h"

namespace g2o {

  using namespace std;

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType, const std::vector<int>& parameterIds) {

    auto it = _vertexToEdgeMap.find(vertexTypes);
    if (it!=_vertexToEdgeMap.end())
      it->second = edgeType;
    else
      _vertexToEdgeMap.insert(make_pair(vertexTypes,EdgeCreatorEntry(edgeType, parameterIds)));
    return true;
  }

  bool EdgeCreator::addAssociation(const std::string& vertexTypes, const std::string& edgeType) {
    return addAssociation(vertexTypes, edgeType, std::vector<int>());
  }

  bool EdgeCreator::removeAssociation(std::string vertexTypes){
    auto it = _vertexToEdgeMap.find(vertexTypes);
    if (it==_vertexToEdgeMap.end())
      return false;
    _vertexToEdgeMap.erase(it);
    return true;
  }


  OptimizableGraph::Edge* EdgeCreator::createEdge(std::vector<OptimizableGraph::Vertex*>& vertices ){
    std::stringstream key;
    Factory* factory=Factory::instance();
    for (auto & vertice : vertices){
      key << factory->tag(vertice) << ";";
    }
    auto it=_vertexToEdgeMap.find(key.str());
    if (it==_vertexToEdgeMap.end()){
      cerr << "no thing in factory: " << key.str() << endl;
      return nullptr;
    }
    HyperGraph::HyperGraphElement* element=factory->construct(it->second._edgeTypeName);
    if (element == nullptr) {
      cerr << "no thing can be created" << endl;
      return nullptr;
    }
    OptimizableGraph::Edge* e = dynamic_cast<OptimizableGraph::Edge*>(element);
    assert(it->second._parameterIds.size() == e->numParameters());
    for (size_t i=0; i<it->second._parameterIds.size(); i++){
      if (! e->setParameterId(i,it->second._parameterIds[i])) {
        cerr << "no thing in good for setting params" << endl;
        return nullptr;
      }
    }
    assert (e);
    for (size_t i=0; i<vertices.size(); i++)
      e->vertices()[i]=vertices[i];
    return e;
  }

} // namespace g2o

