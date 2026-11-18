// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_GRAPH_PARAMETER_CONTAINER_HH_
#define G2O_GRAPH_PARAMETER_CONTAINER_HH_

#include <g2o/core/abi.h>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/parameter.h>
#include <g2o/prefix.h>

#include <iosfwd>
#include <map>
#include <memory>
#include <string>

G2O_START_NAMESPACE

/**
 * \brief map id to parameters
 */
class G2O_CORE_API ParameterContainer
    : protected std::map<int, std::unique_ptr<Parameter>> {
 public:
  typedef std::map<int, std::unique_ptr<Parameter>> BaseClass;

  //! create a container that owns its parameters
  ParameterContainer();
  virtual ~ParameterContainer();
  //! adds the parameter, which the container owns, and frees a parameter whose
  //! id is negative or already in the container
  bool addParameter(std::unique_ptr<Parameter> p);
  //! return a parameter based on its ID
  Parameter* getParameter(int id);
  //! return a parameter based on its ID
  const Parameter* getParameter(int id) const;
  //! remove a parameter from the container and return it to the caller
  std::unique_ptr<Parameter> detachParameter(int id);
  //! read parameters from a stream
  virtual bool read(
      std::istream& is,
      const std::map<std::string, std::string>* renamedMap = nullptr);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  void clear();

  // stuff of the base class that should re-appear
  using BaseClass::size;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
