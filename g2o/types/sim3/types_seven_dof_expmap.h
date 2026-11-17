// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2016-2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_SEVEN_DOF_EXPMAP_TYPES
#define G2O_SEVEN_DOF_EXPMAP_TYPES

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/prefix.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/sim3/g2o_types_sim3_api.h>
#include <g2o/types/sim3/sim3.h>

G2O_START_NAMESPACE

/**
 * \brief Sim3 Vertex, (x,y,z,qw,qx,qy,qz)
 * the parameterization for the increments constructed is a 7d vector
 * (x,y,z,qx,qy,qz) (note that we leave out the w part of the quaternion.
 *
 * Will represent relative transformation between two cameras
 */
class G2O_TYPES_SIM3_API VertexSim3Expmap : public BaseVertex<7, Sim3> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  VertexSim3Expmap();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  virtual void setToOriginImpl() { _estimate = Sim3(); }

  virtual void oplusImpl(const double* update_) {
    Eigen::Map<Vector7d> update(const_cast<double*>(update_));

    if (_fix_scale) update[6] = 0;

    Sim3 s(update);
    setEstimate(s * estimate());
  }

  Vector2D _principle_point1, _principle_point2;
  Vector2D _focal_length1, _focal_length2;

  Vector2D cam_map1(const Vector2D& v) const {
    Vector2D res;
    res[0] = v[0] * _focal_length1[0] + _principle_point1[0];
    res[1] = v[1] * _focal_length1[1] + _principle_point1[1];
    return res;
  }

  Vector2D cam_map2(const Vector2D& v) const {
    Vector2D res;
    res[0] = v[0] * _focal_length2[0] + _principle_point2[0];
    res[1] = v[1] * _focal_length2[1] + _principle_point2[1];
    return res;
  }

  bool _fix_scale;

 protected:
};

/**
 * \brief 7D edge between two Vertex7
 */
class G2O_TYPES_SIM3_API EdgeSim3
    : public BaseBinaryEdge<7, Sim3, VertexSim3Expmap, VertexSim3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
  void computeError() {
    const VertexSim3Expmap* v1 =
        dynamic_cast<const VertexSim3Expmap*>(_vertices[0]);
    const VertexSim3Expmap* v2 =
        dynamic_cast<const VertexSim3Expmap*>(_vertices[1]);

    Sim3 C(_measurement);
    Sim3 error_ = C * v1->estimate() * v2->estimate().inverse();
    _error = error_.log();
  }

  virtual double initialEstimatePossible(const OptimizableGraph::VertexSet&,
                                         OptimizableGraph::Vertex*) {
    return 1.;
  }
  virtual void initialEstimate(const OptimizableGraph::VertexSet& from,
                               OptimizableGraph::Vertex* /*to*/) {
    VertexSim3Expmap* v1 = dynamic_cast<VertexSim3Expmap*>(_vertices[0]);
    VertexSim3Expmap* v2 = dynamic_cast<VertexSim3Expmap*>(_vertices[1]);
    if (from.count(v1) > 0)
      v2->setEstimate(measurement() * v1->estimate());
    else
      v1->setEstimate(measurement().inverse() * v2->estimate());
  }
};

/**/
class G2O_TYPES_SIM3_API EdgeSim3ProjectXYZ
    : public BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSim3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSim3ProjectXYZ();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;

  void computeError() {
    const VertexSim3Expmap* v1 =
        dynamic_cast<const VertexSim3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 =
        dynamic_cast<const VertexSBAPointXYZ*>(_vertices[0]);

    Vector2D obs(_measurement);
    _error = obs - v1->cam_map1(project(v1->estimate().map(v2->estimate())));
  }

  // virtual void linearizeOplus();
};

/**/
class G2O_TYPES_SIM3_API EdgeInverseSim3ProjectXYZ
    : public BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSim3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeInverseSim3ProjectXYZ();
  virtual bool read(std::istream& is);
  virtual bool write(std::ostream& os) const;
  void computeError() {
    const VertexSim3Expmap* v1 =
        static_cast<const VertexSim3Expmap*>(_vertices[1]);
    const VertexSBAPointXYZ* v2 =
        static_cast<const VertexSBAPointXYZ*>(_vertices[0]);

    Vector2D obs(_measurement);
    _error = obs - v1->cam_map2(
                       project(v1->estimate().inverse().map(v2->estimate())));
  }

  // virtual void linearizeOplus();
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif
