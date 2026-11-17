// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "parameter_se2_offset.h"
#include "vertex_se2.h"

G2O_START_NAMESPACE
  namespace tutorial {

    ParameterSE2Offset::ParameterSE2Offset()
    {
    }

    void ParameterSE2Offset::setOffset(const SE2& offset)
    {
      _offset = offset;
      _inverseOffset = offset.inverse();
    }

    bool ParameterSE2Offset::read(std::istream& is)
    {
      double x, y, th;
      is >> x >> y >> th;
      setOffset(SE2(x, y, th));
      return true;
    }

    bool ParameterSE2Offset::write(std::ostream& os) const
    {
      os << _offset.translation().x() << " " << _offset.translation().y() << " " << _offset.rotation().angle();
      return os.good();
    }

    void CacheSE2Offset::updateImpl()
    {
      const VertexSE2* v = dynamic_cast<const VertexSE2*>(vertex());
      _n2w = v->estimate() * _offsetParam->offset();
      _w2n = _n2w.inverse();
    }

    bool CacheSE2Offset::resolveDependancies()
    {
      _offsetParam = dynamic_cast<ParameterSE2Offset*> (_parameters[0]);
      return _offsetParam != nullptr;
    }

  } // end namespace tutorial
G2O_END_NAMESPACE
