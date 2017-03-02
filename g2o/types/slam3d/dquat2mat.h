#ifndef _DQUAT2MAT_H_
#define _DQUAT2MAT_H_

#include <Eigen/Core>

#include <g2o/core/abi.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>

#include <g2o/prefix.hpp>

G2O_START_NAMESPACE
  namespace internal {

    void  G2O_TYPES_SLAM3D_API compute_dq_dR ( Eigen::Matrix<double, 3 , 9, Eigen::ColMajor>&  dq_dR , const double&  r11 , const double&  r21 , const double&  r31 , const double&  r12 , const double&  r22 , const double&  r32 , const double&  r13 , const double&  r23 , const double&  r33 );

    void  G2O_TYPES_SLAM3D_API compute_dR_dq ( Eigen::Matrix<double, 9 , 3, Eigen::ColMajor>&  dR_dq , const double&  qx , const double&  qy , const double&  qz , const double&  qw ) ;
  } // namespace internal
G2O_END_NAMESPACE

#include <g2o/suffix.hpp>

#endif
