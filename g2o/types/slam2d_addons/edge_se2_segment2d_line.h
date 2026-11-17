// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE2_SEGMENT2D_LINE_H
#define G2O_EDGE_SE2_SEGMENT2D_LINE_H

#include <g2o/config.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h>
#include <g2o/types/slam2d_addons/vertex_segment2d.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  class G2O_TYPES_SLAM2D_ADDONS_API EdgeSE2Segment2DLine : public BaseBinaryEdge<2, Vector2D, VertexSE2, VertexSegment2D> //Avoid redefinition of BaseEdge in MSVC
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      EdgeSE2Segment2DLine();

      double theta() const {return _measurement[0];}
      double rho()   const {return _measurement[1];}

      void   setTheta( double t)  {_measurement[0] = t;}
      void   setRho( double r)    {_measurement[1] = r;}


      void computeError()
      {
        const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = dynamic_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2D predP1 = iEst * l2->estimateP1();
        Vector2D predP2 = iEst * l2->estimateP2();
        Vector2D dP = predP2 - predP1;
        Vector2D normal(dP.y(), -dP.x()); normal.normalize();
        Vector2D prediction(atan2(normal.y(), normal.x()),
			    predP1.dot(normal)*.5 + predP2.dot(normal)*.5);

	_error=prediction-_measurement;
        _error[0]=normalize_theta(_error[0]);
      }

      virtual bool setMeasurementData(const double* d){
        Eigen::Map<const Vector2D> data(d);
	_measurement = data;
	return true;
      }

      virtual bool getMeasurementData(double* d) const{
        Eigen::Map<Vector2D> data(d);
	data = _measurement;
	return true;
      }

      virtual int measurementDimension() const {return 2;}

      virtual bool setMeasurementFromState(){
     const VertexSE2* v1 = dynamic_cast<const VertexSE2*>(_vertices[0]);
        const VertexSegment2D* l2 = dynamic_cast<const VertexSegment2D*>(_vertices[1]);
        SE2 iEst=v1->estimate().inverse();
        Vector2D predP1 = iEst * l2->estimateP1();
        Vector2D predP2 = iEst * l2->estimateP2();
        Vector2D dP = predP2 - predP1;
        Vector2D normal(dP.y(), -dP.x()); normal.normalize();
        Vector2D prediction(atan2(normal.y(), normal.x()),
			    predP1.dot(normal)*.5 + predP2.dot(normal)*.5);
	_measurement = prediction;
	return true;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;


/* #ifndef NUMERIC_JACOBIAN_TWO_D_TYPES */
/*       virtual void linearizeOplus(); */
/* #endif */
  };

/*   class EdgeSE2Segment2DLineWriteGnuplotAction: public WriteGnuplotAction { */
/*   public: */
/*     EdgeSE2Segment2DLineWriteGnuplotAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */

/* #ifdef G2O_HAVE_OPENGL */
/*   class EdgeSE2Segment2DLineDrawAction: public DrawAction{ */
/*   public: */
/*     EdgeSE2Segment2DLineDrawAction(); */
/*     virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,  */
/*             HyperGraphElementAction::Parameters* params_); */
/*   }; */
/* #endif */

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
