// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_HYPER_GRAPH_ACTION_H
#define G2O_HYPER_GRAPH_ACTION_H

#include <g2o/core/cache.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/property.h>

#include <iosfwd>
#include <memory>
#include <ostream>
#include <set>
#include <string>
#include <typeinfo>

// define to get verbose output
// #define G2O_DEBUG_ACTIONLIB

#include <g2o/prefix.h>

G2O_START_NAMESPACE

class CacheContainer;
/**
 * \brief Abstract action that operates on an entire graph
 */
class G2O_CORE_API HyperGraphAction {
 public:
  class G2O_CORE_API Parameters {
   public:
    virtual ~Parameters() = default;
  };

  class G2O_CORE_API ParametersIteration : public Parameters {
   public:
    explicit ParametersIteration(int iter);
    int iteration;
  };

  virtual ~HyperGraphAction() = default;

  /**
   * re-implement to carry out an action given the graph
   */
  virtual HyperGraphAction* operator()(const HyperGraph* graph,
                                       Parameters* parameters = nullptr);
};

/**
 * \brief Abstract action that operates on a graph entity
 */
class G2O_CORE_API HyperGraphElementAction {
 public:
  struct G2O_CORE_API Parameters {
    virtual ~Parameters() = default;
  };
  typedef std::map<std::string, std::unique_ptr<HyperGraphElementAction> >
      ActionMap;
  //! an action should be instantiated with the typeid.name of the graph element
  //! on which it operates
  HyperGraphElementAction(const std::string& typeName_ = "");

  //! redefine this to do the action stuff. If successful, the action returns a
  //! pointer to itself
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element, Parameters* parameters);

  //! redefine this to do the action stuff. If successful, the action returns a
  //! pointer to itself
  virtual HyperGraphElementAction* operator()(
      const HyperGraph::HyperGraphElement* element, Parameters* parameters);

  //! destroyed actions release the memory
  virtual ~HyperGraphElementAction() = default;

  //! returns the typeid name of the action
  const std::string& typeName() const { return _typeName; }

  //! returns the name of an action, e.g "draw"
  const std::string& name() const { return _name; }

  //! sets the type on which an action has to operate
  void setTypeName(const std::string& typeName_);

 protected:
  std::string _typeName;
  std::string _name;
};

/**
 * \brief collection of actions
 *
 * collection of actions calls contains homogeneous actions operating on
 * different types all collected actions have the same name and should have the
 * same functionality
 */
class G2O_CORE_API HyperGraphElementActionCollection
    : public HyperGraphElementAction {
 public:
  //! constructor. name_ is the name of the action e.g.draw).
  HyperGraphElementActionCollection(const std::string& name_);
  //! destructor: it deletes all actions in the pool.
  virtual ~HyperGraphElementActionCollection() = default;
  //! calling functions, they return a pointer to the instance of action in
  //! actionMap that was active on element
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element, Parameters* parameters);
  virtual HyperGraphElementAction* operator()(
      const HyperGraph::HyperGraphElement* element, Parameters* parameters);
  ActionMap& actionMap() { return _actionMap; }
  //! inserts an action in the pool. The action should have the same name of the
  //! container. returns false on failure (the container has a different name
  //! than the action);
  bool registerAction(HyperGraphElementAction* action);
  bool unregisterAction(HyperGraphElementAction* action);

 protected:
  ActionMap _actionMap;

 private:
  HyperGraphElementActionCollection(const HyperGraphElementActionCollection&) =
      delete;
  HyperGraphElementActionCollection& operator=(
      const HyperGraphElementActionCollection&) = delete;
};

/**
 * \brief library of actions, indexed by the action name;
 *
 * library of actions, indexed by the action name;
 * one can use ti to register a collection of actions
 */
class G2O_CORE_API HyperGraphActionLibrary {
 public:
  //! return the single instance of the HyperGraphActionLibrary
  static HyperGraphActionLibrary* instance();
  //! free the instance
  G2O_CORE_DEPRECATED static void destroy();

  // returns a pointer to a collection indexed by name
  HyperGraphElementAction* actionByName(const std::string& name);
  // registers a basic action in the pool. If necessary a container is created
  bool registerAction(HyperGraphElementAction* action);
  bool unregisterAction(HyperGraphElementAction* action);

  inline HyperGraphElementAction::ActionMap& actionMap() { return _actionMap; }

 protected:
  HyperGraphActionLibrary() = default;
  ~HyperGraphActionLibrary() = default;
  HyperGraphElementAction::ActionMap _actionMap;

 private:
  HyperGraphActionLibrary(const HyperGraphActionLibrary&) = delete;
  HyperGraphActionLibrary& operator=(const HyperGraphActionLibrary&) = delete;
};

/**
 * apply an action to all the elements of the graph.
 */
void G2O_CORE_API applyAction(
    HyperGraph* graph, HyperGraphElementAction* action,
    HyperGraphElementAction::Parameters* parameters = nullptr,
    const std::string& typeName = "");

/**
 * brief write into gnuplot
 */
class G2O_CORE_API WriteGnuplotAction : public HyperGraphElementAction {
 public:
  struct G2O_CORE_API Parameters : public HyperGraphElementAction::Parameters {
    std::ostream* os;
  };
  WriteGnuplotAction(const std::string& typeName_);
};

/**
 * \brief draw actions
 */

class G2O_CORE_API DrawAction : public HyperGraphElementAction {
 public:
  class G2O_CORE_API Parameters : public HyperGraphElementAction::Parameters,
                                  public PropertyMap {
   public:
    Parameters() = default;
  };
  DrawAction(const std::string& typeName_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  void initializeDrawActionsCache();
  void drawCache(CacheContainer* caches,
                 HyperGraphElementAction::Parameters* params_);
  void drawUserData(HyperGraph::Data* data,
                    HyperGraphElementAction::Parameters* params_);
  Parameters* _previousParams;
  BoolProperty* _show{};
  BoolProperty* _showId{};
  HyperGraphElementAction* _cacheDrawActions;
};

template <typename T>
class RegisterActionProxy {
 public:
  RegisterActionProxy() {
#ifdef G2O_DEBUG_ACTIONLIB
    ngx::Log("Registering action of type {}", typeid(T).name());
#endif
    _action.reset(new T());
    HyperGraphActionLibrary::instance()->registerAction(_action.get());
  }

  ~RegisterActionProxy() {
#ifdef G2O_DEBUG_ACTIONLIB
    ngx::Log("Unregistering action of type {}", typeid(T).name());
#endif
    HyperGraphActionLibrary::instance()->unregisterAction(_action.get());
  }

 private:
  std::unique_ptr<HyperGraphElementAction> _action;
};

#define G2O_REGISTER_ACTION(classname)            \
  extern "C" void g2o_action_##classname(void) {} \
  static g2o::RegisterActionProxy<classname> g_action_proxy_##classname;
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
