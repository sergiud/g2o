// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_DEPRECATED_VERTEX_TRACKXYZ_H_
#define G2O_DEPRECATED_VERTEX_TRACKXYZ_H_

#include <g2o/core/base_vertex.h>
#include <g2o/core/hyper_graph_action.h>

#include <g2o/deprecated/g2o_deprecated_types_slam3d_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE
namespace deprecated {

  /**
   * Vertex for a tracked point in space
   */
  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZ : public BaseVertex<3, Eigen::Vector3d>
    {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW    
          VertexPointXYZ() {}
        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual void setToOriginImpl() { _estimate.fill(0.); }

        virtual void oplusImpl(const double* update_) {
          Eigen::Map<const Eigen::Vector3d> update(update_);
          _estimate += update;
        }

        virtual bool setEstimateDataImpl(const double* est){
          Eigen::Map<const Eigen::Vector3d> _est(est);
          _estimate = _est;
          return true;
        }

        virtual bool getEstimateData(double* est) const{
          Eigen::Map<Eigen::Vector3d> _est(est);
          _est = _estimate;
          return true;
        }

        virtual int estimateDimension() const {
          return 3;
        }

        virtual bool setMinimalEstimateDataImpl(const double* est){
          _estimate = Eigen::Map<const Eigen::Vector3d>(est);
          return true;
        }

        virtual bool getMinimalEstimateData(double* est) const{
          Eigen::Map<Eigen::Vector3d> v(est);
          v = _estimate;
          return true;
        }

        virtual int minimalEstimateDimension() const {
          return 3;
        }

    };

  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZWriteGnuplotAction: public WriteGnuplotAction
  {
    public:
      VertexPointXYZWriteGnuplotAction();
      virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ );
  };

#ifdef G2O_HAVE_OPENGL
  class G2O_DEPRECATED_TYPES_SLAM3D_API VertexPointXYZDrawAction: public DrawAction{
  public:
    VertexPointXYZDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element, 
            HyperGraphElementAction::Parameters* params_);

    
  protected:
    FloatProperty *_pointSize;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };
#endif

}
}
#endif
