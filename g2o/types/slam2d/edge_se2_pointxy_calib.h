// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE2_XY_CALIB_H
#define G2O_EDGE_SE2_XY_CALIB_H

#include <g2o/core/base_multi_edge.h>
#include <g2o/types/slam2d/g2o_types_slam2d_api.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d/vertex_se2.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief Landmark measurement that also calibrates an offset for the landmark measurement
   */
  class G2O_TYPES_SLAM2D_API EdgeSE2PointXYCalib : public BaseMultiEdge<2, Vector2D>
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2PointXYCalib();

      void computeError()
      {
        const VertexSE2* v1     = dynamic_cast<const VertexSE2*>(_vertices[0]);
        const VertexPointXY* l2 = dynamic_cast<const VertexPointXY*>(_vertices[1]);
        const VertexSE2* calib  = dynamic_cast<const VertexSE2*>(_vertices[2]);
        _error = ((v1->estimate() * calib->estimate()).inverse() * l2->estimate()) - _measurement;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);}
      virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);
  };

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
