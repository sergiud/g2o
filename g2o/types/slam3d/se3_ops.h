// SPDX-FileCopyrightText: 2011 H. Strasdat
// SPDX-FileCopyrightText: 2017 Sergiu Deitsch
//
// SPDX-License-Identifier: BSD-2-Clause

#ifndef G2O_MATH_STUFF
#define G2O_MATH_STUFF

#include <g2o/core/eigen_types.h>
#include <g2o/prefix.h>
#include <g2o/types/slam3d/g2o_types_slam3d_api.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

G2O_START_NAMESPACE

inline G2O_TYPES_SLAM3D_API Matrix3D skew(const Vector3D& v);
inline G2O_TYPES_SLAM3D_API Vector3D deltaR(const Matrix3D& R);
inline G2O_TYPES_SLAM3D_API Vector2D project(const Vector3D&);
inline G2O_TYPES_SLAM3D_API Vector3D project(const Vector4D&);
inline G2O_TYPES_SLAM3D_API Vector3D unproject(const Vector2D&);
/**
 * Rotation into the frame of a plane, whose last row is the unit normal of the
 * plane and whose first two rows span the plane.
 */
inline G2O_TYPES_SLAM3D_API Matrix3D frameOfNormal(const Vector3D& normal);
inline G2O_TYPES_SLAM3D_API Vector4D unproject(const Vector3D&);

#include "se3_ops.hpp"

G2O_END_NAMESPACE

#include <g2o/suffix.h>

#endif  // MATH_STUFF
