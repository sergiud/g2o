// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_ELLIPSE_H
#define G2O_VERTEX_ELLIPSE_H

#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/prefix.h>
#include <g2o/types/data/robot_data.h>

G2O_START_NAMESPACE

/**
 * \brief string ellipse to be attached to a vertex
 */
class G2O_TYPES_DATA_API VertexEllipse : public RobotData {
 public:
  typedef std::vector<Vector2F, Eigen::aligned_allocator<Vector2F> >
      myVector2fVector;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexEllipse();
  ~VertexEllipse();

  virtual bool write(std::ostream& os) const;
  virtual bool read(std::istream& is);

  const Matrix3F& covariance() { return _covariance; }
  void setCovariance(Matrix3F& c) {
    _covariance = c;
    _updateSVD();
  }
  const Matrix2F& U() { return _UMatrix; }
  const Vector2F& singularValues() { return _singularValues; }

  const myVector2fVector& matchingVertices() { return _matchingVertices; }
  void addMatchingVertex(float x, float y) {
    Vector2F v(x, y);
    _matchingVertices.push_back(v);
  }

  void clearMatchingVertices() { _matchingVertices.clear(); }

  const std::vector<int>& matchingVerticesIDs() { return _matchingVerticesIDs; }
  void addMatchingVertexID(int id) { _matchingVerticesIDs.push_back(id); }
  void clearMatchingVerticesIDs() { _matchingVerticesIDs.clear(); }

 protected:
  void _updateSVD() const;
  Matrix3F _covariance;
  mutable Matrix2F _UMatrix;
  mutable Vector2F _singularValues;
  std::vector<int> _matchingVerticesIDs;
  myVector2fVector _matchingVertices;
};

#ifdef G2O_HAVE_OPENGL
class G2O_TYPES_DATA_API VertexEllipseDrawAction : public DrawAction {
 public:
  VertexEllipseDrawAction();
  virtual HyperGraphElementAction* operator()(
      HyperGraph::HyperGraphElement* element,
      HyperGraphElementAction::Parameters* params_);

 protected:
  virtual bool refreshPropertyPtrs(
      HyperGraphElementAction::Parameters* params_);
  DoubleProperty* _scaleFactor;
};
#endif

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
