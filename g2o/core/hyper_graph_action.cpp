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

#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/cache.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/stuff/macros.h>


#include <iostream>

namespace g2o {
  using namespace std;

  HyperGraphAction::ParametersIteration::ParametersIteration(int iter) :
    HyperGraphAction::Parameters(),
    iteration(iter)
  {
  }

  HyperGraphAction* HyperGraphAction::operator()(const HyperGraph*, Parameters*)
  {
    return nullptr;
  }

  HyperGraphElementAction::HyperGraphElementAction(const std::string& typeName_)
  {
    _typeName = typeName_;
  }

  void HyperGraphElementAction::setTypeName(const std::string& typeName_)
  {
    _typeName = typeName_;
  }


  HyperGraphElementAction* HyperGraphElementAction::operator()(HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return nullptr;
  }

  HyperGraphElementAction* HyperGraphElementAction::operator()(const HyperGraph::HyperGraphElement* , HyperGraphElementAction::Parameters* )
  {
    return nullptr;
  }

  HyperGraphElementActionCollection::HyperGraphElementActionCollection(const std::string& name_)
  {
    _name = name_;
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    auto it=_actionMap.find(typeid(*element).name());
    //cerr << typeid(*element).name() << endl;
    if (it==_actionMap.end())
      return nullptr;
    HyperGraphElementAction* action=it->second.get();
    return (*action)(element, params);
  }

  HyperGraphElementAction* HyperGraphElementActionCollection::operator()(const HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params)
  {
    auto it=_actionMap.find(typeid(*element).name());
    if (it==_actionMap.end())
      return nullptr;
    HyperGraphElementAction* action=it->second.get();
    return (*action)(element, params);
  }

  bool HyperGraphElementActionCollection::registerAction(HyperGraphElementAction* action)
  {
#  ifdef G2O_DEBUG_ACTIONLIB
    cerr << __PRETTY_FUNCTION__ << " " << action->name() << " " << action->typeName() << endl;
#  endif
    if (action->name()!=name()){
      cerr << __PRETTY_FUNCTION__  << ": invalid attempt to register an action in a collection with a different name " <<  name() << " " << action->name() << endl;
    }
    _actionMap.insert(std::make_pair( action->typeName(), std::unique_ptr<HyperGraphElementAction>(action)));
    return true;
  }

  bool HyperGraphElementActionCollection::unregisterAction(HyperGraphElementAction* action)
  {
    for (auto it=_actionMap.begin(); it != _actionMap.end(); ++it) {
      if (it->second.get() == action){
         it->second.release();
        _actionMap.erase(it);
        return true;
      }
    }
    return false;
  }

  HyperGraphActionLibrary* HyperGraphActionLibrary::instance()
  {
    static HyperGraphActionLibrary lib;
    return &lib;
  }

  void HyperGraphActionLibrary::destroy()
  {
  }

  HyperGraphElementAction* HyperGraphActionLibrary::actionByName(const std::string& name)
  {
    auto it=_actionMap.find(name);
    if (it!=_actionMap.end())
      return it->second.get();
    return nullptr;
  }

  bool HyperGraphActionLibrary::registerAction(HyperGraphElementAction* action)
  {
    HyperGraphElementAction* oldAction = actionByName(action->name());
    HyperGraphElementActionCollection* collection = nullptr;
    if (oldAction != nullptr) {
      collection = dynamic_cast<HyperGraphElementActionCollection*>(oldAction);
      if (collection == nullptr) {
        cerr << __PRETTY_FUNCTION__ << ": fatal error, a collection is not at the first level in the library" << endl;
        return 0;
      }
    }
    if (collection == nullptr) {
#ifdef G2O_DEBUG_ACTIONLIB
      cerr << __PRETTY_FUNCTION__ << ": creating collection for \"" << action->name() << "\"" << endl;
#endif
      std::unique_ptr<HyperGraphElementActionCollection> col(new HyperGraphElementActionCollection(action->name()));
      collection = col.get();
      _actionMap.insert(make_pair(action->name(), std::move(col)));
    }
    return collection->registerAction(action);
  }

  bool HyperGraphActionLibrary::unregisterAction(HyperGraphElementAction* action)
  {
    std::list<std::string> collectionDeleteList;

    // Search all the collections and delete the registered actions; if a collection becomes empty, schedule it for deletion; note that we can't delete the collections as we go because this will screw up the state of the iterators
    for (auto & it : _actionMap) {
      HyperGraphElementActionCollection* collection = dynamic_cast<HyperGraphElementActionCollection*> (it.second.get());
      if (auto collection = dynamic_cast<HyperGraphElementActionCollection*> (it.second.get())) {
        collection->unregisterAction(action);
        if (collection->actionMap().empty()) {
          it.second.reset();
          collectionDeleteList.push_back(it.first);
        }
      }
    }

    // Delete any empty action collections
    for (auto& col : collectionDeleteList) {
      //cout << "Deleting collection " << (*itc)->name() << endl;
      _actionMap.erase(col);
    }

    return true;
  }


  WriteGnuplotAction::WriteGnuplotAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_)
  {
    _name="writeGnuplot";
  }

  DrawAction::DrawAction(const std::string& typeName_)
    : HyperGraphElementAction(typeName_)
  {
    _name="draw";
    _previousParams = (Parameters*)0x42;
    refreshPropertyPtrs(nullptr);
    _cacheDrawActions = nullptr;
  }

  bool DrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (_previousParams == params_)
      return false;
    DrawAction::Parameters* p=dynamic_cast<DrawAction::Parameters*>(params_);
    if (p == nullptr){
      _previousParams = nullptr;
      _show = nullptr;
      _showId = nullptr;
    } else {
      _previousParams = p;
      _show = p->makeProperty<BoolProperty>(_typeName+"::SHOW", true);
      _showId = p->makeProperty<BoolProperty>(_typeName+"::SHOW_ID", false);
    }
    return true;
  }

  void DrawAction::initializeDrawActionsCache() {
    if (_cacheDrawActions == nullptr){
      _cacheDrawActions = HyperGraphActionLibrary::instance()->actionByName("draw");
    }
  }

  void DrawAction::drawCache(CacheContainer* caches, HyperGraphElementAction::Parameters* params_) {
    if (caches != nullptr){
      for (auto & cache : *caches){
        Cache* c = cache.second;
        (*_cacheDrawActions)(c, params_);
      }
    }
  }

  void DrawAction::drawUserData(HyperGraph::Data* data, HyperGraphElementAction::Parameters* params_){
    while ((data != nullptr) && (_cacheDrawActions != nullptr) ){
      (*_cacheDrawActions)(data, params_);
      data=data->next();
    }
  }

  void applyAction(HyperGraph* graph, HyperGraphElementAction* action, HyperGraphElementAction::Parameters* params, const std::string& typeName)
  {
    for (auto & it : graph->vertices()){
      if ( typeName.empty() || typeid(*it.second).name()==typeName){
        (*action)(it.second, params);
      }
    }
    for (auto it : graph->edges()){
      if ( typeName.empty() || typeid(*it).name()==typeName)
        (*action)(it, params);
    }
  }

} // namespace g2o
