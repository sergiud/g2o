// SPDX-FileCopyrightText: 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_VERTEX_SE3_EULER_
#define G2O_VERTEX_SE3_EULER_

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h>

#include <g2o/prefix.h>

G2O_START_NAMESPACE

/**
 * \brief 3D pose Vertex, (x,y,z,roll,pitch,yaw)
 * the internal parameterization is the same as veretx_se3_quat. 
 * Only the read/write operations are rewritten to input/output euler angles.
 */
class G2O_TYPES_SLAM3D_ADDONS_API VertexSE3Euler : public VertexSE3
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual bool read(std::istream& is);
    virtual bool write(std::ostream& os) const;
};

G2O_END_NAMESPACE

#include <g2o/suffix.h>


#endif
