// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_ODOM_DIFFERENTIAL_PARAMS_H
#define G2O_VERTEX_ODOM_DIFFERENTIAL_PARAMS_H

#include <g2o/core/base_vertex.h>
#include <g2o/prefix.h>
#include <g2o/types/sclam2d/edge_se2_sensor_calib.h>
#include <g2o/types/sclam2d/g2o_types_sclam2d_api.h>

G2O_START_NAMESPACE

class G2O_TYPES_SCLAM2D_API VertexOdomDifferentialParams
    : public BaseVertex<3, Vector3D> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexOdomDifferentialParams() = default;
  virtual void setToOriginImpl() { _estimate << 1., 1., 1.; }

  virtual void oplusImpl(const double* v) {
    for (int i = 0; i < 3; i++) _estimate(i) += v[i];
  }

  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
