// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_TUTORIAL_VERTEX_POINT_XY_H
#define G2O_TUTORIAL_VERTEX_POINT_XY_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/examples/tutorial_slam2d/g2o_tutorial_slam2d_api.h>

#include <Eigen/Core>

#include <g2o/prefix.h>

G2O_START_NAMESPACE
  namespace tutorial {

    class G2O_TUTORIAL_SLAM2D_API VertexPointXY : public BaseVertex<2, Eigen::Vector2d>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPointXY();

        virtual void setToOriginImpl() {
          _estimate.setZero();
        }

        virtual void oplusImpl(const double* update)
        {
          _estimate[0] += update[0];
          _estimate[1] += update[1];
        }

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

    };

  } // namespace tutorial
G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
