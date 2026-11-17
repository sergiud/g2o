// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include "vertex_point_xy.h"

using namespace Eigen;

G2O_START_NAMESPACE
  namespace tutorial {

    VertexPointXY::VertexPointXY() :
      BaseVertex<2, Vector2d>()
    {
      _estimate.setZero();
    }

    bool VertexPointXY::read(std::istream& is)
    {
      is >> _estimate[0] >> _estimate[1];
      return true;
    }

    bool VertexPointXY::write(std::ostream& os) const
    {
      os << estimate()(0) << " " << estimate()(1);
      return os.good();
    }

  } // namespace tutorial
G2O_END_NAMESPACE
