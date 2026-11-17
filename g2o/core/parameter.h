// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_GRAPH_PARAMETER_HH_
#define G2O_GRAPH_PARAMETER_HH_

#include <g2o/core/hyper_graph.h>
#include <g2o/prefix.h>

#include <iosfwd>

G2O_START_NAMESPACE

class G2O_CORE_API Parameter : public HyperGraph::HyperGraphElement {
 public:
  Parameter();
  virtual ~Parameter() = default;
  ;
  //! read the data from a stream
  virtual bool read(std::istream& is) = 0;
  //! write the data to a stream
  virtual bool write(std::ostream& os) const = 0;
  int id() const { return _id; }
  void setId(int id_);
  virtual HyperGraph::HyperGraphElementType elementType() const {
    return HyperGraph::HGET_PARAMETER;
  }

 protected:
  int _id;
};

typedef std::vector<Parameter*> ParameterVector;

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
