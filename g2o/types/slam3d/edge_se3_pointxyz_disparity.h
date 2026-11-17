// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_
#define G2O_EDGE_SE3_POINTXYZ_DISPARITY_H_

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/types/slam3d/parameter_camera.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>

#define EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief edge from a track to a depth camera node using a disparity measurement
   *
   * the disparity measurement is normalized: disparity / (focal_x * baseline)
   */
  // first two args are the measurement type, second two the connection classes
  class G2O_TYPES_SLAM3D_API EdgeSE3PointXYZDisparity : public BaseBinaryEdge<3, Vector3D, VertexSE3, VertexPointXYZ> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeSE3PointXYZDisparity();
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;

    // return the error estimate as a 3-vector
    void computeError();

#ifdef EDGE_PROJECT_DISPARITY_ANALYTIC_JACOBIAN
    virtual void linearizeOplus();
#endif

    virtual void setMeasurement(const Vector3D& m){
      _measurement = m;
    }

    virtual bool setMeasurementData(const double* d){
      Eigen::Map<const Vector3D> v(d);
      _measurement = v;
      return true;
    }

    virtual bool getMeasurementData(double* d) const{
      Eigen::Map<Vector3D> v(d);
      v=_measurement;
      return true;
    }
    
    virtual int measurementDimension() const {return 3;}

    virtual bool setMeasurementFromState() ;
    
    virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& from, 
             OptimizableGraph::Vertex* to) { 
      (void) to; 
      return (from.count(_vertices[0]) == 1 ? 1.0 : -1.0);
    }

    virtual void initialEstimate(const OptimizableGraph::VertexSet& from, OptimizableGraph::Vertex* to);

    const ParameterCamera* cameraParameter() const { return params; }
  private:
    Eigen::Matrix<double,3,9,Eigen::ColMajor> J; // jacobian before projection
    virtual bool resolveCaches();
    ParameterCamera* params{};
    CacheCamera* cache{};
  };


#ifdef G2O_HAVE_OPENGL
  class G2O_TYPES_SLAM3D_API EdgeProjectDisparityDrawAction: public DrawAction{
  public:
    EdgeProjectDisparityDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);
  };
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
