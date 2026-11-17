// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2012 R. Kuemmerle
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_STRUCTURE_ONLY_SOLVER_H
#define G2O_STRUCTURE_ONLY_SOLVER_H

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/sparse_optimizer.h>

G2O_START_NAMESPACE

/**
 * \brief This is a solver for "structure-only" optimization"
 *
 * Given the problem of landmark-based SLAM or bundle adjustment, this class
 * performs optimization on the landmarks while the poses are kept fixed. This
 * can be done very efficiently, since the position on the landmarks are
 * indepdented given the poses are known.
 *
 * This class slightly misuses the API of g2o. It is designed in a way, it can
 * work on the very same graph which reflects the problem of landmark-based
 * SLAM, bundle adjustment and which is meant to be solved using the Schur
 * complement. Thus, it can be called just before or after joint-optimization
 * without the need of additional setup. Call calc() with the point features you
 * want to optimize.
 *
 * This class is still considered as being experimentally!
 */
template <int PointDoF>
class StructureOnlySolver : public OptimizationAlgorithm {
 public:
  StructureOnlySolver() { _verbose = true; }

  virtual OptimizationAlgorithm::SolverResult solve(int iteration,
                                                    bool online = false) {
    (void)iteration;
    (void)online;
    return calc(_points, 1);
  }

  OptimizationAlgorithm::SolverResult calc(
      OptimizableGraph::VertexContainer& vertices, int num_iters,
      int num_max_trials = 10) {
    JacobianWorkspace auxWorkspace;
    auxWorkspace.updateSize(2, 50);
    auxWorkspace.allocate();

    for (auto& vertice : vertices) {
      bool stop = false;
      g2o::OptimizableGraph::Vertex* v =
          dynamic_cast<OptimizableGraph::Vertex*>(vertice);
      assert(v->dimension() == PointDoF);
      g2o::HyperGraph::EdgeSet& track = v->edges();
      assert(track.size() >= 2);
      double chi2 = 0;
      // TODO make these parameters
      double mu = 0.01;
      double nu = 2;

      for (auto it_t : track) {
        g2o::OptimizableGraph::Edge* e =
            dynamic_cast<g2o::OptimizableGraph::Edge*>(it_t);
        e->computeError();
        chi2 += e->chi2();
      }

      if (v->fixed() == false) {
        Eigen::Matrix<double, PointDoF, PointDoF, Eigen::ColMajor> H_pp;
        H_pp.resize(v->dimension(), v->dimension());
        v->mapHessianMemory(H_pp.data());
        for (int i_g = 0; i_g < num_iters; ++i_g) {
          H_pp.setZero();
          v->clearQuadraticForm();

          g2o::HyperGraph::EdgeSet& track = v->edges();
          assert(track.size() >= 1);

          for (auto it_t : track) {
            g2o::OptimizableGraph::Edge* e =
                dynamic_cast<g2o::OptimizableGraph::Edge*>(it_t);

            // fix all the other vertices and remember their fix value
            std::vector<bool> remember_fix_status(e->vertices().size());

            for (size_t k = 0; k < e->vertices().size(); ++k) {
              OptimizableGraph::Vertex* otherV =
                  dynamic_cast<OptimizableGraph::Vertex*>(e->vertex(k));
              if (otherV != v) {
                remember_fix_status[k] = otherV->fixed();
                otherV->setFixed(true);
              }
            }

            // build the matrix
            e->computeError();
            e->linearizeOplus(auxWorkspace);
            e->constructQuadraticForm();

            // Restore frame's initial fixed() values
            for (size_t k = 0; k < e->vertices().size(); ++k) {
              OptimizableGraph::Vertex* otherV =
                  dynamic_cast<g2o::OptimizableGraph::Vertex*>(e->vertex(k));
              if (otherV != v) {
                otherV->setFixed(remember_fix_status[k]);
              }
            }
          }

          Eigen::Map<Eigen::Matrix<double, PointDoF, 1, Eigen::ColMajor> > b(
              v->bData(), v->dimension());

          if (b.norm() < 0.001) {
            stop = true;
            break;
          }

          int trial = 0;
          do {
            Eigen::Matrix<double, PointDoF, PointDoF, Eigen::ColMajor> H_pp_mu =
                H_pp;
            H_pp_mu.diagonal().array() += mu;
            Eigen::LDLT<
                Eigen::Matrix<double, PointDoF, PointDoF, Eigen::ColMajor> >
                chol_H_pp(H_pp_mu);
            bool goodStep = false;
            if (chol_H_pp.isPositive()) {
              Eigen::Matrix<double, PointDoF, 1, Eigen::ColMajor> delta_p =
                  chol_H_pp.solve(b);
              v->push();
              v->oplus(delta_p.data());
              double new_chi2 = 0.;
              for (auto it_t : track) {
                g2o::OptimizableGraph::Edge* e =
                    dynamic_cast<g2o::OptimizableGraph::Edge*>(it_t);
                e->computeError();
                new_chi2 += e->chi2();
              }
              assert(g2o_isnan(new_chi2) == false && "Chi is NaN");
              double rho = (chi2 - new_chi2);
              if (rho > 0 && g2o_isfinite(new_chi2)) {
                goodStep = true;
                chi2 = new_chi2;
                v->discardTop();
              } else {
                goodStep = false;
                v->pop();
              }
            }

            // update the damping factor based on the result of the last
            // increment
            if (goodStep) {
              mu *= 1. / 3.;
              nu = 2.;
              trial = 0;
              break;
            } else {
              mu *= nu;
              nu *= 2.;
              ++trial;
              if (trial >= num_max_trials) {
                stop = true;
                break;
              }
            }
          } while (!stop);
          if (stop) break;
        }
      }
    }
    return OK;
  }

  virtual bool init(bool) {
    // collect the vertices
    _points.clear();
    for (auto it = optimizer()->activeVertices().begin();
         it != optimizer()->activeVertices().end(); ++it) {
      OptimizableGraph::Vertex* v = *it;
      if (v->marginalized()) {
        _points.push_back(v);
      }
    }
    return true;
  }

  virtual bool computeMarginals(SparseBlockMatrix<MatrixXD>&,
                                const std::vector<std::pair<int, int> >&) {
    return false;
  }

  virtual bool updateStructure(const std::vector<HyperGraph::Vertex*>&,
                               const HyperGraph::EdgeSet&) {
    return true;
  }

  //! return the points of the optimization problem
  OptimizableGraph::VertexContainer& points() { return _points; }
  const OptimizableGraph::VertexContainer& points() const { return _points; }

 protected:
  bool _verbose;
  OptimizableGraph::VertexContainer _points;
};

G2O_END_NAMESPACE

#endif
