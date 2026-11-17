// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <cstdio>

#ifdef G2O_HAVE_OPENGL
#include <g2o/stuff/opengl_primitives.h>
#include <g2o/stuff/opengl_wrapper.h>
#endif

#include <typeinfo>

G2O_START_NAMESPACE

  bool VertexPointXYZ::read(std::istream& is) {
    Vector3D lv;
    for (int i=0; i<3; i++)
      is >> lv[i];
    setEstimate(lv);
    return true;
  }

  bool VertexPointXYZ::write(std::ostream& os) const {
    Vector3D lv=estimate();
    for (int i=0; i<3; i++){
      os << lv[i] << " ";
    }
    return os.good();
  }


#ifdef G2O_HAVE_OPENGL
  VertexPointXYZDrawAction::VertexPointXYZDrawAction(): DrawAction(typeid(VertexPointXYZ).name()){
  }

  bool VertexPointXYZDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams != nullptr){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = nullptr;
    }
    return true;
  }


  HyperGraphElementAction* VertexPointXYZDrawAction::operator()(HyperGraph::HyperGraphElement* element, 
                     HyperGraphElementAction::Parameters* params ){

    if (typeid(*element).name()!=_typeName)
      return nullptr;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params);
    if (_previousParams == nullptr)
      return this;
    
    if ((_show != nullptr) && !_show->value())
      return this;
    VertexPointXYZ* that = dynamic_cast<VertexPointXYZ*>(element);
    

    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(LANDMARK_VERTEX_COLOR);
    float ps = _pointSize != nullptr ? _pointSize->value() :  1.f;
    glTranslatef((float)that->estimate()(0),(float)that->estimate()(1),(float)that->estimate()(2));
    opengl::drawPoint(ps);
    glPopAttrib();
    drawCache(that->cacheContainer(), params);
    drawUserData(that->userData(), params);
    glPopMatrix();
    return this;
  }
#endif

  VertexPointXYZWriteGnuplotAction::VertexPointXYZWriteGnuplotAction() :
    WriteGnuplotAction(typeid(VertexPointXYZ).name())
  {
  }

  HyperGraphElementAction* VertexPointXYZWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_ )
  {
    if (typeid(*element).name()!=_typeName)
      return nullptr;
    WriteGnuplotAction::Parameters* params=dynamic_cast<WriteGnuplotAction::Parameters*>(params_);
    if (params->os == nullptr){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified" << std::endl;
      return nullptr;
    }

    VertexPointXYZ* v = dynamic_cast<VertexPointXYZ*>(element);
    *(params->os) << v->estimate().x() << " " << v->estimate().y() << " " << v->estimate().z() << " " << std::endl;
    return this;
  }

G2O_END_NAMESPACE

