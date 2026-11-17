// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H
#define G2O_TUTORIAL_PARAMETER_SE2_OFFSET_H

#include <g2o/core/cache.h>
#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>
#include <g2o/prefix.h>

#include "se2.h"

G2O_START_NAMESPACE
namespace tutorial {

class G2O_TUTORIAL_SLAM2D_API ParameterSE2Offset : public Parameter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  ParameterSE2Offset();

  void setOffset(const SE2& offset = SE2());

  const SE2& offset() const { return _offset; }
  const SE2& inverseOffset() const { return _inverseOffset; }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

 protected:
  SE2 _offset;
  SE2 _inverseOffset;
};

class G2O_TUTORIAL_SLAM2D_API CacheSE2Offset : public Cache {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  const SE2& w2n() const { return _w2n; }
  const SE2& n2w() const { return _n2w; }

 protected:
  virtual void updateImpl();
  virtual bool resolveDependancies();

  ParameterSE2Offset* _offsetParam{};
  SE2 _w2n, _n2w;
};

}  // end namespace tutorial
G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
