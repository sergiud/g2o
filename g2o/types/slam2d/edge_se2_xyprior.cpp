// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017-2025 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam2d/edge_se2_xyprior.h>

G2O_START_NAMESPACE

  EdgeSE2XYPrior::EdgeSE2XYPrior() : BaseUnaryEdge< 2, Vector2D, g2o::VertexSE2 >()
  {
    
  }

  bool EdgeSE2XYPrior::read(std::istream& is)
  {
    Vector2D p;
    is >> p[0] >> p[1];
    setMeasurement(p);    
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j) {
        is >> information()(i, j);
        if (!is) return false;
        if (i != j)
          information()(j, i) = information()(i, j);
      }
    return true;
  }

  bool EdgeSE2XYPrior::write(std::ostream& os) const
  {
    Vector2D p = measurement();
    os << p[0] << " " << p[1];
    for (int i = 0; i < 2; ++i)
      for (int j = i; j < 2; ++j)
        os << " " << information()(i, j);
    return os.good();
  }

  void EdgeSE2XYPrior::linearizeOplus()
  {
    _jacobianOplusXi << 1,0,0, 0,1,0;
  }

G2O_END_NAMESPACE
