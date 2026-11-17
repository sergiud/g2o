// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_CREATORS_H
#define G2O_CREATORS_H

#include <g2o/core/hyper_graph.h>
#include <g2o/prefix.h>

#include <string>
#include <typeinfo>

G2O_START_NAMESPACE

/**
 * \brief Abstract interface for allocating HyperGraphElement
 */
class G2O_CORE_API AbstractHyperGraphElementCreator {
 public:
  /**
   * create a hyper graph element. Has to implemented in derived class.
   */
  virtual HyperGraph::HyperGraphElement* construct() = 0;
  /**
   * name of the class to be created. Has to implemented in derived class.
   */
  virtual const std::string& name() const = 0;

  virtual ~AbstractHyperGraphElementCreator() = default;
};

/**
 * \brief templatized creator class which creates graph elements
 */
template <typename T>
class HyperGraphElementCreator : public AbstractHyperGraphElementCreator {
 public:
  HyperGraphElementCreator() : _name(typeid(T).name()) {}
#if defined(_WIN32) && \
    defined(__GNUC__)  // force stack alignment on Windows with GCC
  __attribute__((force_align_arg_pointer))
#endif
  HyperGraph::HyperGraphElement* construct() {
    return new T;
  }
  virtual const std::string& name() const { return _name; }

 protected:
  std::string _name;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
