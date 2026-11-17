// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_TAG_H
#define G2O_VERTEX_TAG_H

#include <g2o/core/hyper_graph_action.h>
#include <g2o/types/data/g2o_types_data_api.h>
#include <g2o/types/data/robot_data.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

  /**
   * \brief string tag to be attached to a vertex
   *
   * A laser measurement obtained by a robot. The measurement is equipped with a pose of the robot at which
   * the measurement was taken. The read/write function correspond to the CARMEN logfile format.
   */
  class G2O_TYPES_DATA_API VertexTag : public RobotData
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexTag();
      ~VertexTag();

      virtual bool write(std::ostream& os) const;
      virtual bool read(std::istream& is);
      
      const std::string name() const { return _name;} 
      void setName(const std::string& name_) {_name=name_;}
      const Vector3F& position() const {return _position;}
      void setPosition( const Vector3F& p) {_position = p;}
    protected:
      std::string _name;
      Vector3F _position;
      Vector3F _odom2d;
  };

 #ifdef G2O_HAVE_OPENGL 
  class G2O_TYPES_DATA_API VertexTagDrawAction: public DrawAction{
  public:
    VertexTagDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_ );
  protected:
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
    DoubleProperty* _textSize{};
  };
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
