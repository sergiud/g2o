// SPDX-FileCopyrightText: 2012 Rainer Kuemmerle, Giorgio Grisetti
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#include <g2o/apps/g2o_cli/dl_wrapper.h>
#include <g2o/apps/g2o_cli/g2o_common.h>
#include <g2o/apps/g2o_cli/output_helper.h>
#include <g2o/core/estimate_propagator.h>
#include <g2o/core/factory.h>
#include <g2o/core/hyper_dijkstra.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/stuff/color_macros.h>
#include <g2o/stuff/command_args.h>
#include <g2o/stuff/filesys_tools.h>
#include <g2o/stuff/logger.h>
#include <g2o/stuff/macros.h>
#include <g2o/stuff/streamed.h>
#include <g2o/stuff/string_tools.h>
#include <g2o/stuff/timeutil.h>
#include <g2o/stuff/unscented.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include "edge_creator.h"
#include "edge_labeler.h"
#include "star.h"

using namespace std;
using namespace g2o;
using namespace Eigen;

typedef SigmaPoint<VectorXd> MySigmaPoint;

void testMarginals(SparseOptimizer& optimizer) {
  ngx::Log("Projecting marginals");
  std::vector<std::pair<int, int> > blockIndices;
  for (auto v : optimizer.activeVertices()) {
    if (v->hessianIndex() >= 0) {
      blockIndices.push_back(make_pair(v->hessianIndex(), v->hessianIndex()));
    }
    // if (v->hessianIndex()>0){
    //   blockIndices.push_back(make_pair(v->hessianIndex()-1,
    //   v->hessianIndex()));
    // }
  }
  SparseBlockMatrix<MatrixXd> spinv;
  if (optimizer.computeMarginals(spinv, blockIndices)) {
    for (auto v : optimizer.activeVertices()) {
      ngx::Log("Vertex id:{}", v->id());
      if (v->hessianIndex() >= 0) {
        ngx::Log("increments block :{}, {} covariance:", v->hessianIndex(),
                 v->hessianIndex());
        VectorXd mean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        mean.fill(0);
        VectorXd oldMean(
            v->minimalEstimateDimension());  // HACK: need to set identity
        v->getMinimalEstimateData(&oldMean[0]);
        MatrixXd& cov = *(spinv.block(v->hessianIndex(), v->hessianIndex()));
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
        ngx::Log("{}", Streamed(cov));
        if (!sampleUnscented(spts, mean, cov)) continue;

        // now apply the oplus operator to the sigma points,
        // and get the points in the global space
        std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> >
            tspts = spts;

        for (size_t j = 0; j < spts.size(); j++) {
          v->push();
          // cerr << "v_before [" << j << "]" << endl;
          v->getMinimalEstimateData(&mean[0]);
          // cerr << mean << endl;
          // cerr << "sigma [" << j << "]" << endl;
          // cerr << spts[j]._sample << endl;
          v->oplus(&(spts[j]._sample[0]));
          v->getMinimalEstimateData(&mean[0]);
          tspts[j]._sample = mean;
          // cerr << "oplus [" << j << "]" << endl;
          // cerr << tspts[j]._sample << endl;
          v->pop();
        }
        MatrixXd cov2 = cov;
        reconstructGaussian(mean, cov2, tspts);
        ngx::Log("global block :{}, {}", v->hessianIndex(), v->hessianIndex());
        ngx::Log("mean: ");
        ngx::Log("{}", Streamed(mean));
        ngx::Log("oldMean: ");
        ngx::Log("{}", Streamed(oldMean));
        ngx::Log("cov: ");
        ngx::Log("{}", Streamed(cov2));
      }
      // if (v->hessianIndex()>0){
      //   cerr << "inv block :" << v->hessianIndex()-1 << ", " <<
      //   v->hessianIndex()<< endl; cerr << *(spinv.block(v->hessianIndex()-1,
      //   v->hessianIndex())); cerr << endl;
      // }
    }
  }
}

int unscentedTest() {
  MatrixXd m = MatrixXd(6, 6);
  for (int i = 0; i < 6; i++) {
    for (int j = i; j < 6; j++) {
      m(i, j) = m(j, i) = i * j + 1;
    }
  }
  m += MatrixXd::Identity(6, 6);
  ngx::Log("{}", Streamed(m));
  VectorXd mean(6);
  mean.fill(1);

  std::vector<MySigmaPoint, Eigen::aligned_allocator<MySigmaPoint> > spts;
  sampleUnscented(spts, mean, m);
  for (size_t i = 0; i < spts.size(); i++) {
    ngx::Log("Point {} \nwi={} wp={} ", i, spts[i]._wi, spts[i]._wp);
    ngx::Log("{}", Streamed(spts[i]._sample));
  }

  VectorXd recMean(6);
  MatrixXd recCov(6, 6);

  reconstructGaussian(recMean, recCov, spts);

  ngx::Log("recMean");
  ngx::Log("{}", Streamed(recMean));

  ngx::Log("recCov");
  ngx::Log("{}", Streamed(recCov));

  return 0;
}
