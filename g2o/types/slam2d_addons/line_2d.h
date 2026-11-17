// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_LINE2D_H
#define G2O_LINE2D_H

#include <g2o/types/slam2d/se2.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <Eigen/Core>

G2O_START_NAMESPACE

  struct Line2D : public Vector2D{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Line2D() {
      setZero();
    }
    Line2D(const Vector2D& v) {
      (*this)(0) = v(0);
      (*this)(1) = v(1);
    }
  };

  inline Line2D operator * (const SE2 & t, const Line2D& l){
    Line2D est = l;
    est[0] += t.rotation().angle();
    est[0] = normalize_theta(est[0]);
    Vector2D n(cos(est[0]), sin(est[0]));
    est[1] += n.dot(t.translation());
    return est;
  }

G2O_END_NAMESPACE

#endif
