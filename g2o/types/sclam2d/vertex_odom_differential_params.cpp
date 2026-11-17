// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/sclam2d/vertex_odom_differential_params.h>

G2O_START_NAMESPACE
  bool VertexOdomDifferentialParams::read(std::istream& is)
  {
    is >> _estimate(0) >> _estimate(1) >> _estimate(2);
    return true;
  }

  bool VertexOdomDifferentialParams::write(std::ostream& os) const
  {
    os << estimate()(0) << " " << estimate()(1) << " " << estimate()(2);
    return os.good();
  }

G2O_END_NAMESPACE
