// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_LINE2D_POINTXY_H
#define G2O_EDGE_LINE2D_POINTXY_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/stuff/misc.h>
#include <g2o/types/slam2d/vertex_point_xy.h>
#include <g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h>
#include <g2o/types/slam2d_addons/vertex_line2d.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXY : public BaseBinaryEdge<1, double, VertexLine2D, VertexPointXY> //Avoid redefinition of BaseEdge in MSVC
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeLine2DPointXY() = default;

      void computeError()
      {
        const VertexLine2D* l = dynamic_cast<const VertexLine2D*>(_vertices[0]);
        const VertexPointXY* p = dynamic_cast<const VertexPointXY*>(_vertices[1]);
        Vector2D n(cos(l->theta()), sin(l->theta()));
        double prediction=n.dot(p->estimate())-l->rho();
        _error[0] =  prediction - _measurement;
      }

      virtual bool setMeasurementData(const double* d){
	_measurement = *d;
        return true;
      }

      virtual bool getMeasurementData(double* d) const{
        *d = _measurement;
        return true;
      }

      virtual int measurementDimension() const {return 1;}

      virtual bool setMeasurementFromState(){
        const VertexLine2D* l = dynamic_cast<const VertexLine2D*>(_vertices[0]);
        const VertexPointXY* p = dynamic_cast<const VertexPointXY*>(_vertices[1]);
        Vector2D n(cos(l->theta()), sin(l->theta()));
        double prediction=n.dot(p->estimate())-l->rho();
	_measurement = prediction;
        return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      /* virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to); */
      /* virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to) { (void) to; return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);} */
/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXYWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeLine2DPointXYWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class G2O_TYPES_SLAM2D_ADDONS_API EdgeLine2DPointXYDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeLine2DPointXYDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
