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

#include <g2o/core/optimizable_graph.h>
#include <g2o/types/slam2d/edge_se2_offset.h>
#include <g2o/types/slam2d/parameter_se2_offset.h>

#include <iostream>

G2O_START_NAMESPACE
  using namespace std;

  EdgeSE2Offset::EdgeSE2Offset() : BaseBinaryEdge<3, SE2, VertexSE2, VertexSE2>() {
    information().setIdentity();
    _offsetFrom = nullptr;
    _offsetTo = nullptr;
    _cacheFrom = nullptr;
    _cacheTo = nullptr;
    resizeParameters(2);
    installParameter(_offsetFrom, 0);
    installParameter(_offsetTo, 1);
  }

  bool EdgeSE2Offset::resolveCaches(){
    assert(_offsetFrom && _offsetTo);

    ParameterVector pv(1);
    pv[0]=_offsetFrom;
    resolveCache(_cacheFrom, dynamic_cast<OptimizableGraph::Vertex*>(_vertices[0]),"CACHE_SE2_OFFSET",pv);
    pv[0]=_offsetTo;
    resolveCache(_cacheTo, dynamic_cast<OptimizableGraph::Vertex*>(_vertices[1]),"CACHE_SE2_OFFSET",pv);
    return ((_cacheFrom != nullptr) && (_cacheTo != nullptr));
  }

  bool EdgeSE2Offset::read(std::istream& is) {
    int pidFrom, pidTo;
    is >> pidFrom >> pidTo;
    if (! setParameterId(0,pidFrom))
      return false;
    if (! setParameterId(1,pidTo))
      return false;

    Vector3D meas;
    for (int i=0; i<3; i++)
      is >> meas[i];
    setMeasurement(SE2(meas));
    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix with the Identity
      information().setIdentity();
    }
    return true;
  }

  bool EdgeSE2Offset::write(std::ostream& os) const {
    os << _offsetFrom->id() << " " << _offsetTo->id() << " ";
    for (int i=0; i<3; i++) os  << measurement()[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }

  void EdgeSE2Offset::computeError() {
    SE2 delta=_inverseMeasurement * _cacheFrom->w2n() * _cacheTo->n2w();
    _error.head<2>() = delta.translation();
    _error(2)=delta.rotation().angle();
  }

  bool EdgeSE2Offset::setMeasurementFromState(){
    SE2 delta = _cacheFrom->w2n() * _cacheTo->n2w();
    setMeasurement(delta);
    return true;
  }

  void EdgeSE2Offset::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE2 *from = dynamic_cast<VertexSE2*>(_vertices[0]);
    VertexSE2 *to   = dynamic_cast<VertexSE2*>(_vertices[1]);

    SE2 virtualMeasurement = _cacheFrom->offsetParam()->offset() * measurement() * _cacheTo->offsetParam()->offset().inverse();

    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * virtualMeasurement);
    } else
      from->setEstimate(to->estimate() * virtualMeasurement.inverse());
  }

G2O_END_NAMESPACE
