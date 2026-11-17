// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/config.h>
#include <g2o/types/data/vertex_tag.h>
#include <g2o/stuff/macros.h>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_wrapper.h>
#include <g2o/stuff/opengl_primitives.h>
#include <EXTERNAL/freeglut/freeglut_minimal.h>
#endif

#include <iomanip>
using namespace std;

G2O_START_NAMESPACE

  VertexTag::VertexTag() : RobotData()
  {
  }

  VertexTag::~VertexTag()
  = default;

  bool VertexTag::read(std::istream& is)
  {
    is >> _name;
    is >> _position.x() >> _position.y() >> _position.z();
    is >> _odom2d.x() >> _odom2d.y() >> _odom2d.z();
    is >> _timestamp;
    is >> _hostname;
    is >> _loggerTimestamp;
    return true;
  }

  bool VertexTag::write(std::ostream& os) const
  {
    os << _name << " ";
    os << FIXED(_position.x() << " " << _position.y() << " " << _position.z() << " ");
    os << FIXED(_odom2d.x() << " " << _odom2d.y() << " " << _odom2d.z() << " ");
    os << FIXED(" " << timestamp() << " " << hostname() << " " << loggerTimestamp());
    return os.good();
  }



#ifdef G2O_HAVE_OPENGL
  VertexTagDrawAction::VertexTagDrawAction(): DrawAction(typeid(VertexTag).name()){
  }

  bool VertexTagDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (!DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams != nullptr){
      _textSize = _previousParams->makeProperty<DoubleProperty>(_typeName + "::TEXT_SIZE", 1);
    } else {
      _textSize = nullptr;
    }
    return true;
  }

  HyperGraphElementAction* VertexTagDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                 HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return nullptr;

    refreshPropertyPtrs(params_);
    if (_previousParams == nullptr){
      return this;
    }
    VertexTag* that = dynamic_cast<VertexTag*>(element);

    glPushMatrix();
    glColor3f(1.f,0.2f,1.f);
    glTranslatef(that->position().x(), that->position().y(), that->position().z());
    float textSize = 1;
    if (_textSize != nullptr )
      textSize = (float)_textSize->value();
    opengl::drawBox(0.1f*textSize, 0.1f*textSize, 0.1f*textSize);
    glTranslatef(0.2f*textSize, 0.f, 0.f);
    glScalef(0.003f*textSize,0.003f*textSize,1.f);
    freeglut_minimal::glutStrokeString(freeglut_minimal::GLUT_STROKE_ROMAN, that->name().c_str());
    glPopMatrix();
    return this;
  }
#endif

G2O_END_NAMESPACE
